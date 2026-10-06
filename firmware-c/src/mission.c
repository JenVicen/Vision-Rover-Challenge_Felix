#include "mission.h"

#include "config.h"
#include "control.h"
#include "esp_link.h"
#include "geom.h"
#include "motors.h"
#include "planner.h"
#include "sensors.h"
#include "status_led.h"
#include "vision_client.h"
#include "world.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mission";

/* --------------------------------------------------------------------
 * Estado interno
 * -------------------------------------------------------------------- */

static mission_state_t s_state = MS_WAIT_TELEMETRY;
static cube_color_t s_target_color = COLOR_NONE;
static float s_target_distance = 1e6f;
static int64_t s_state_entered_ms = 0;
static int64_t s_task_started_ms = 0;
static bool s_round_started = false;

/* Objetivo de aproximacion previo al empuje. */
static float s_stage_col = 0.0f;
static float s_stage_row = 0.0f;

/* Desvio fijo mientras el obstaculo conserve su posicion. */
static bool s_detouring = false;
static float s_detour_col = 0.0f;
static float s_detour_row = 0.0f;
static cube_color_t s_detour_blocker_color = COLOR_NONE;
static float s_detour_blocker_col = 0.0f;
static float s_detour_blocker_row = 0.0f;
static float s_align_cube_col = 0.0f;
static float s_align_cube_row = 0.0f;
static bool s_yield_goal_valid = false;
static float s_yield_goal_col = 0.0f;
static float s_yield_goal_row = 0.0f;

/* Ultimo plan calculado. */
static plan_t s_plan;

/* Estado del control de progreso y de los reintentos por color. */
static int64_t s_progress_last_ms = 0;
static float s_progress_last_dist = 1e6f;
static int s_progress_fails = 0;
static int64_t s_deferred_until_ms[COLOR_COUNT] = {0};
static float s_deferred_cube_col[COLOR_COUNT] = {0};
static float s_deferred_cube_row[COLOR_COUNT] = {0};
static int s_task_failures[COLOR_COUNT] = {0};

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

const char *mission_state_name(mission_state_t state)
{
    switch (state)
    {
    case MS_WAIT_TELEMETRY:
        return "WAIT_TELEMETRY";
    case MS_WAIT_START:
        return "WAIT_START";
    case MS_SELECT:
        return "SELECT";
    case MS_GO_STAGE:
        return "GO_STAGE";
    case MS_ALIGN:
        return "ALIGN";
    case MS_PUSH:
        return "PUSH";
    case MS_VERIFY:
        return "VERIFY";
    case MS_YIELD:
        return "YIELD";
    case MS_DONE:
        return "DONE";
    default:
        return "?";
    }
}

static void enter_state(mission_state_t next)
{
    if (next == s_state)
    {
        return;
    }
    ESP_LOGI(TAG, "%s -> %s", mission_state_name(s_state), mission_state_name(next));
    s_state = next;
    s_state_entered_ms = now_ms();
    control_reset();
}

static int64_t time_in_state(void)
{
    return now_ms() - s_state_entered_ms;
}

/* --------------------------------------------------------------------
 * Geometria del empuje
 * --------------------------------------------------------------------
 * Para llevar un cubo a su deposito, el rover se coloca sobre la recta
 * cubo->deposito, del lado contrario al deposito, y luego avanza en linea
 * recta atravesando la posicion del cubo.
 *
 *      deposito  <----  cubo  <----  [ROVER]
 *                              stage point
 */
static void compute_stage_point(const cube_obs_t *cube, const depot_t *depot,
                                float *out_col, float *out_row)
{
    const float dc = depot->col - cube->col;
    const float dr = depot->row - cube->row;
    const float len = sqrtf(dc * dc + dr * dr);

    if (len < 0.001f)
    {
        /* El cubo ya esta sobre el centro del deposito: no hay direccion util. */
        *out_col = cube->col;
        *out_row = cube->row;
        return;
    }

    const float ux = dc / len;
    const float uy = dr / len;

    *out_col = cube->col - ux * STAGE_DISTANCE_CELLS;
    *out_row = cube->row - uy * STAGE_DISTANCE_CELLS;
}

/* Mantiene un punto dentro del campo con un margen para el cuerpo del rover. */
static void clamp_to_field(const world_t *w, float *col, float *row)
{
    const float margin = ROVER_RADIUS_CELLS;
    *col = clampf(*col, margin, (float)w->grid_cols - margin);
    *row = clampf(*row, margin, (float)w->grid_rows - margin);
}

/* Valida si existe un punto de aproximacion util dentro del campo. */
static bool stage_point_feasible(const world_t *w, const cube_obs_t *cube,
                                 const depot_t *depot)
{
    float raw_col, raw_row;
    compute_stage_point(cube, depot, &raw_col, &raw_row);
    float safe_col = raw_col;
    float safe_row = raw_row;
    clamp_to_field(w, &safe_col, &safe_row);
    return dist_cells(raw_col, raw_row, safe_col, safe_row) <=
           PUSH_LATERAL_MAX_CELLS;
}

/* Distancia minima entre un punto y un segmento. */
static float point_segment_distance(float point_col, float point_row,
                                    float from_col, float from_row,
                                    float to_col, float to_row)
{
    const float vx = to_col - from_col;
    const float vy = to_row - from_row;
    const float len_sq = vx * vx + vy * vy;

    if (len_sq < 0.001f)
    {
        return dist_cells(from_col, from_row, point_col, point_row);
    }

    float t = ((point_col - from_col) * vx +
               (point_row - from_row) * vy) /
              len_sq;
    t = clampf(t, 0.0f, 1.0f);

    const float closest_col = from_col + t * vx;
    const float closest_row = from_row + t * vy;

    return dist_cells(closest_col, closest_row, point_col, point_row);
}

static const cube_obs_t *find_push_corridor_blocker(const world_t *w,
                                                    const cube_obs_t *target,
                                                    const depot_t *depot)
{
    /* Dos cubos de 60 mm necesitan unos tres cuadros entre centros para no
     * tocarse; se agrega media celda contra ruido de vision. */
    const float clearance = (w->cube_side > 0.0f ? w->cube_side : 3.0f) + 0.5f;
    for (int i = 0; i < w->cube_count; ++i)
    {
        const cube_obs_t *candidate = &w->cubes[i];
        if (!candidate->present || candidate->color == target->color ||
            candidate->age_ms > OBSERVATION_STALE_MS)
        {
            continue;
        }
        if (point_segment_distance(candidate->col, candidate->row,
                                   target->col, target->row,
                                   depot->col, depot->row) < clearance)
        {
            return candidate;
        }
    }
    return NULL;
}

static bool task_feasible(const world_t *w, const cube_obs_t *cube,
                          const depot_t *depot)
{
    return find_push_corridor_blocker(w, cube, depot) == NULL;
}

static void defer_edge_task(const cube_obs_t *cube)
{
    if (cube->color <= COLOR_NONE || cube->color >= COLOR_COUNT)
    {
        return;
    }
    s_deferred_until_ms[cube->color] = now_ms() + EDGE_RETRY_MS;
    s_deferred_cube_col[cube->color] = cube->col;
    s_deferred_cube_row[cube->color] = cube->row;
}

static void defer_failed_task(const cube_obs_t *cube)
{
    if (cube->color <= COLOR_NONE || cube->color >= COLOR_COUNT)
    {
        return;
    }
    ++s_task_failures[cube->color];
    int64_t delay_ms =
        (int64_t)TASK_RETRY_BASE_MS * s_task_failures[cube->color];
    if (delay_ms > TASK_RETRY_MAX_MS)
    {
        delay_ms = TASK_RETRY_MAX_MS;
    }
    s_deferred_until_ms[cube->color] = now_ms() + delay_ms;
    s_deferred_cube_col[cube->color] = cube->col;
    s_deferred_cube_row[cube->color] = cube->row;
    ESP_LOGW(TAG, "aplazo %s por %lld ms tras %d fallos",
             color_name(cube->color), (long long)delay_ms,
             s_task_failures[cube->color]);
}

static bool edge_task_deferred(const cube_obs_t *cube)
{
    if (cube->color <= COLOR_NONE || cube->color >= COLOR_COUNT)
    {
        return false;
    }
    if (now_ms() >= s_deferred_until_ms[cube->color])
    {
        return false;
    }
    if (dist_cells(cube->col, cube->row,
                   s_deferred_cube_col[cube->color],
                   s_deferred_cube_row[cube->color]) >=
        EDGE_RETRY_MOVE_CELLS)
    {
        s_deferred_until_ms[cube->color] = 0;
        s_task_failures[cube->color] = 0;
        return false;
    }
    return true;
}

/*
 * Si la recta hacia el punto de empuje pasa por encima del cubo, ir directo
 * lo desplazaria antes de tiempo. En ese caso se inserta un desvio lateral.
 */
static bool path_crosses_cube(const rover_obs_t *me, float goal_col, float goal_row,
                              const cube_obs_t *cube)
{
    const float distance = point_segment_distance(
        cube->col, cube->row, me->col, me->row, goal_col, goal_row);

    /* Radio del rover + media diagonal aproximada del cubo de 60 mm. */
    const float clearance = ROVER_RADIUS_CELLS + 2.2f;
    return distance < clearance;
}

static const cube_obs_t *find_blocking_cube(const world_t *w,
                                            const rover_obs_t *me,
                                            float goal_col, float goal_row)
{
    const cube_obs_t *blocking = NULL;
    float best_distance = 1e6f;

    for (int i = 0; i < w->cube_count; ++i)
    {
        const cube_obs_t *candidate = &w->cubes[i];
        if (!candidate->present ||
            candidate->age_ms > OBSERVATION_STALE_MS)
        {
            continue;
        }

        const float distance = point_segment_distance(
            candidate->col, candidate->row,
            me->col, me->row, goal_col, goal_row);
        if (path_crosses_cube(me, goal_col, goal_row, candidate) &&
            distance < best_distance)
        {
            blocking = candidate;
            best_distance = distance;
        }
    }

    return blocking;
}

static void compute_detour_point(const world_t *w, const rover_obs_t *me,
                                 const cube_obs_t *cube, const depot_t *depot,
                                 float stage_col, float stage_row,
                                 float *out_col, float *out_row)
{
    /* El eje cubo->deposito no cambia cuando el rover se mueve. Sus dos
     * perpendiculares producen waypoints FIJOS a ambos lados del cubo. */
    const float dc = depot->col - cube->col;
    const float dr = depot->row - cube->row;
    const float len = sqrtf(dc * dc + dr * dr);

    if (len < 0.001f)
    {
        *out_col = cube->col;
        *out_row = cube->row;
        return;
    }

    const float px = -dr / len;
    const float py = dc / len;
    const float offset = ROVER_RADIUS_CELLS * 2.5f;

    float a_col = cube->col + px * offset;
    float a_row = cube->row + py * offset;
    float b_col = cube->col - px * offset;
    float b_row = cube->row - py * offset;
    clamp_to_field(w, &a_col, &a_row);
    clamp_to_field(w, &b_col, &b_row);

    /* Recortar contra un borde puede acercar un candidato nuevamente al
     * cubo. Evaluamos los DOS tramos de cada ruta (rover->desvio->stage),
     * no solo que el waypoint sea cercano. Primero preferimos una ruta cuya
     * separacion minima sea segura; entre rutas equivalentes, la mas corta. */
    const float clearance = ROVER_RADIUS_CELLS + 2.2f;
    const float a_min_clearance = fminf(
        point_segment_distance(cube->col, cube->row,
                               me->col, me->row, a_col, a_row),
        point_segment_distance(cube->col, cube->row,
                               a_col, a_row, stage_col, stage_row));
    const float b_min_clearance = fminf(
        point_segment_distance(cube->col, cube->row,
                               me->col, me->row, b_col, b_row),
        point_segment_distance(cube->col, cube->row,
                               b_col, b_row, stage_col, stage_row));
    const bool a_safe = a_min_clearance >= clearance;
    const bool b_safe = b_min_clearance >= clearance;
    const float a_length = dist_cells(me->col, me->row, a_col, a_row) +
                           dist_cells(a_col, a_row, stage_col, stage_row);
    const float b_length = dist_cells(me->col, me->row, b_col, b_row) +
                           dist_cells(b_col, b_row, stage_col, stage_row);

    const bool choose_a = (a_safe != b_safe)
                              ? a_safe
                              : ((a_safe && b_safe)
                                     ? (a_length <= b_length)
                                     : (a_min_clearance >= b_min_clearance));

    if (choose_a)
    {
        *out_col = a_col;
        *out_row = a_row;
    }
    else
    {
        *out_col = b_col;
        *out_row = b_row;
    }
}

/* --------------------------------------------------------------------
 * Seleccion de tarea y coordinacion
 * -------------------------------------------------------------------- */

static bool observation_fresh(uint32_t age_ms)
{
    return age_ms <= OBSERVATION_STALE_MS;
}

/*
 * Arbitraje del reparto. Los dos rovers ejecutan exactamente el mismo
 * razonamiento sobre la misma informacion, asi que llegan a la misma
 * conclusion sin necesidad de un protocolo de negociacion con turnos.
 */
static bool peer_outranks_us(const peer_status_t *peer, cube_color_t color,
                             float my_distance)
{
    if (!peer->valid || peer->claimed_color != color)
    {
        return false;
    }
    /* Quien ya esta empujando no suelta el cubo. */
    if (peer->pushing)
    {
        return true;
    }
    if (peer->distance_to_target < my_distance - 0.5f)
    {
        return true;
    }
    if (fabsf(peer->distance_to_target - my_distance) <= 0.5f)
    {
        return peer->rover_id < ROVER_ID; /* desempate deterministico */
    }
    return false;
}

/*
 * Elige la tarea resolviendo el reparto OPTIMO de forma exacta (planner.c) y
 * quedandose con la primera de las tareas asignadas a este rover.
 *
 * El planificador ya reparte el trabajo entre los dos rovers usando la
 * telemetria global, que es identica para ambos, asi que normalmente no hay
 * conflicto. peer_outranks_us() solo interviene cuando los dos han llegado a
 * conclusiones distintas -- por ejemplo si uno perdio un frame de camara o el
 * otro ya se comprometio con un cubo antes de que el plan cambiara.
 */
static cube_color_t select_task(const world_t *w, const rover_obs_t *me,
                                const peer_status_t *peer, float *out_distance)
{
    *out_distance = 1e6f;

    if (!planner_solve(w, ROVER_ID, PEER_ID, &s_plan) || s_plan.length == 0)
    {
        return COLOR_NONE;
    }

    /* Recorremos nuestra secuencia y tomamos la primera tarea que el otro
     * rover no nos este disputando con mejor derecho. */
    for (int i = 0; i < s_plan.length; ++i)
    {
        const cube_color_t color = s_plan.sequence[i];

        const cube_obs_t *cube = world_find_cube(w, color);
        const depot_t *depot = world_find_depot(w, color);
        if (cube == NULL || depot == NULL)
        {
            continue;
        }
        if (edge_task_deferred(cube))
        {
            continue;
        }
        if (!task_feasible(w, cube, depot))
        {
            continue;
        }

        float stage_col, stage_row;
        compute_stage_point(cube, depot, &stage_col, &stage_row);
        const float approach = dist_cells(me->col, me->row, stage_col, stage_row);

        if (peer_outranks_us(peer, color, approach))
        {
            continue;
        }

        *out_distance = approach;
        return color;
    }

    return COLOR_NONE;
}

/*
 * Coste, en segundos, de la tarea que estamos ejecutando ahora mismo.
 * Se usa para decidir si un plan nuevo justifica abandonar el actual.
 */
static float current_task_cost(const world_t *w, const rover_obs_t *me)
{
    if (s_target_color == COLOR_NONE)
    {
        return 1e6f;
    }
    const cube_obs_t *cube = world_find_cube(w, s_target_color);
    if (cube == NULL)
    {
        return 1e6f;
    }
    return planner_task_cost(w, me->col, me->row, me->theta, cube,
                             NULL, NULL, NULL);
}

/* --------------------------------------------------------------------
 * Verificacion de progreso
 * -------------------------------------------------------------------- */

static void progress_reset(void)
{
    s_progress_last_ms = 0;
    s_progress_last_dist = 1e6f;
    s_progress_fails = 0;
}

/* Devuelve true si el empuje NO esta progresando y hay que abortar. */
static bool push_stalled(const cube_obs_t *cube, const depot_t *depot)
{
    const float d = dist_cells(cube->col, cube->row, depot->col, depot->row);
    const int64_t t = now_ms();

    if (s_progress_last_ms == 0)
    {
        s_progress_last_ms = t;
        s_progress_last_dist = d;
        return false;
    }

    if (t - s_progress_last_ms < PROGRESS_CHECK_MS)
    {
        return false;
    }

    const float gained = s_progress_last_dist - d;
    s_progress_last_ms = t;
    s_progress_last_dist = d;

    if (gained < PROGRESS_MIN_CELLS)
    {
        ++s_progress_fails;
        ESP_LOGW(TAG, "empuje sin progreso (%.2f celdas en %d ms), fallo %d/%d",
                 gained, (int)PROGRESS_CHECK_MS, s_progress_fails,
                 PROGRESS_MAX_FAILS);
        return s_progress_fails >= PROGRESS_MAX_FAILS;
    }

    s_progress_fails = 0;
    return false;
}

/*
 * Cedemos el paso si el otro rover esta cerca Y tiene prioridad.
 * Prioridad: el que ya esta empujando; si ninguno empuja, el ID menor.
 * Reglas simetricas -> nunca se bloquean los dos a la vez.
 */
static bool must_yield(const rover_obs_t *me, const peer_status_t *peer)
{
    if (!peer->valid)
    {
        return false;
    }

    const float separation = dist_cells(me->col, me->row, peer->col, peer->row);
    if (separation > PEER_AVOID_CELLS)
    {
        return false;
    }

    if (peer->pushing)
    {
        return true;
    }
    return peer->rover_id < ROVER_ID;
}

static void compute_yield_point(const world_t *w, const rover_obs_t *me,
                                const peer_status_t *peer,
                                const rover_obs_t *peer_obs,
                                float *out_col, float *out_row)
{
    if (peer->pushing && peer_obs != NULL &&
        observation_fresh(peer_obs->age_ms))
    {
        float forward_col, forward_row;
        heading_vector(peer_obs->theta, &forward_col, &forward_row);
        const float side_col = -forward_row;
        const float side_row = forward_col;
        float a_col = me->col + side_col * YIELD_MOVE_CELLS;
        float a_row = me->row + side_row * YIELD_MOVE_CELLS;
        float b_col = me->col - side_col * YIELD_MOVE_CELLS;
        float b_row = me->row - side_row * YIELD_MOVE_CELLS;
        clamp_to_field(w, &a_col, &a_row);
        clamp_to_field(w, &b_col, &b_row);

        const bool a_clear =
            find_blocking_cube(w, me, a_col, a_row) == NULL;
        const bool b_clear =
            find_blocking_cube(w, me, b_col, b_row) == NULL;
        if (!a_clear && !b_clear)
        {
            *out_col = me->col;
            *out_row = me->row;
            return;
        }
        const float a_move = dist_cells(me->col, me->row, a_col, a_row);
        const float b_move = dist_cells(me->col, me->row, b_col, b_row);
        const bool choose_a = (a_clear != b_clear) ? a_clear : (a_move >= b_move);

        *out_col = choose_a ? a_col : b_col;
        *out_row = choose_a ? a_row : b_row;
        return;
    }

    const float dx = peer->col - me->col;
    const float dy = peer->row - me->row;
    const float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.001f)
    {
        *out_col = me->col;
        *out_row = me->row;
        return;
    }

    /* Alejarse radialmente aumenta la separacion de forma monotona. Si el
     * borde recorta el punto, al alcanzarlo se recalcula otro desplazamiento. */
    *out_col = me->col - (dx / len) * YIELD_MOVE_CELLS;
    *out_row = me->row - (dy / len) * YIELD_MOVE_CELLS;
    clamp_to_field(w, out_col, out_row);

    const float current_separation =
        dist_cells(me->col, me->row, peer->col, peer->row);
    if (dist_cells(*out_col, *out_row, peer->col, peer->row) <
            current_separation + 1.0f ||
        find_blocking_cube(w, me, *out_col, *out_row) != NULL)
    {
        const float px = -dy / len;
        const float py = dx / len;
        float a_col = me->col + px * YIELD_MOVE_CELLS;
        float a_row = me->row + py * YIELD_MOVE_CELLS;
        float b_col = me->col - px * YIELD_MOVE_CELLS;
        float b_row = me->row - py * YIELD_MOVE_CELLS;
        clamp_to_field(w, &a_col, &a_row);
        clamp_to_field(w, &b_col, &b_row);
        const bool a_clear =
            find_blocking_cube(w, me, a_col, a_row) == NULL;
        const bool b_clear =
            find_blocking_cube(w, me, b_col, b_row) == NULL;
        if (!a_clear && !b_clear)
        {
            *out_col = me->col;
            *out_row = me->row;
        }
        else if (a_clear && (!b_clear ||
                             dist_cells(a_col, a_row, peer->col, peer->row) >=
                                 dist_cells(b_col, b_row, peer->col, peer->row)))
        {
            *out_col = a_col;
            *out_row = a_row;
        }
        else
        {
            *out_col = b_col;
            *out_row = b_row;
        }
    }
}

/*
 * Cortesia durante el empuje propio. Si los cuerpos se aproximan desde
 * cualquier angulo, el ID mayor se detiene y el menor conserva el paso.
 */
static bool must_yield_during_push(const rover_obs_t *me,
                                   const peer_status_t *peer)
{
    if (!peer->valid)
    {
        return false;
    }
    if (ROVER_ID < peer->rover_id)
    {
        return false; /* el de ID menor mantiene su paso */
    }
    return dist_cells(me->col, me->row, peer->col, peer->row) <=
           PUSH_CROSS_AVOID_CELLS;
}

/* --------------------------------------------------------------------
 * Seguridad local
 * -------------------------------------------------------------------- */

/*
 * El ultrasonico se lee solo cuando NO estamos empujando: durante el empuje
 * el cubo esta pegado al frente y dispararia el freno continuamente.
 */
static bool obstacle_too_close(void)
{
    const float cm = ultrasonic_read_cm();
    return (cm > 0.0f) && (cm < ULTRA_EMERGENCY_CM);
}

/* Cerca del punto de staging, el ultrasonico puede ver el propio cubo que se
 * va a empujar. No debe bloquear la aproximacion si la vision confirma que el
 * eco esta exactamente al frente y a la distancia esperada del objetivo. */
static bool target_cube_explains_obstacle(const rover_obs_t *me,
                                          const cube_obs_t *cube)
{
    const float distance = dist_cells(me->col, me->row, cube->col, cube->row);
    if (distance > STAGE_DISTANCE_CELLS + ROVER_RADIUS_CELLS)
    {
        return false;
    }

    const float bearing = bearing_deg(me->col, me->row, cube->col, cube->row);
    return fabsf(wrap180(bearing - me->theta)) <= REALIGN_THRESHOLD_DEG;
}

/* --------------------------------------------------------------------
 * Bucle de mision
 * -------------------------------------------------------------------- */

static void mission_step(void)
{
    world_t w;

    /* ---- 1. Telemetria viva? ---- */
    if (!vision_get_world(&w) || vision_age_ms() > TELEMETRY_TIMEOUT_MS)
    {
        motors_stop();
        LED_STOPPED();
        enter_state(MS_WAIT_TELEMETRY);
        return;
    }

    const rover_obs_t *me = world_find_rover(&w, ROVER_ID);
    if (me == NULL || !observation_fresh(me->age_ms))
    {
        /* La camara nos perdio. Parar es mas seguro que navegar a ciegas:
         * el campo es de 1 m y un movimiento sin correccion se sale rapido. */
        motors_stop();
        LED_STOPPED();
        return;
    }

    peer_status_t peer = esp_link_peer();
    const rover_obs_t *peer_obs = world_find_rover(&w, PEER_ID);
    if (peer_obs != NULL && observation_fresh(peer_obs->age_ms))
    {
        /* La vision da posicion a 20 Hz y sirve de respaldo si se pierde un
         * anuncio ESP-NOW. El estado pushing se conserva cuando el enlace si
         * esta vivo; solo se actualiza la geometria con la fuente mas fresca. */
        peer.rover_id = PEER_ID;
        peer.col = peer_obs->col;
        peer.row = peer_obs->row;
        peer.valid = true;
    }

    /* ---- 2. Arranque de la ronda ---- */
    if (!s_round_started)
    {
        if (w.phase == PHASE_RUNNING || button_pressed())
        {
            s_round_started = true;
            ESP_LOGI(TAG, "ronda iniciada (phase=%s)", phase_name(w.phase));
            enter_state(MS_SELECT);
        }
        else
        {
            motors_stop();
            LED_WAITING();
            enter_state(MS_WAIT_START);
            return;
        }
    }

    if (w.phase == PHASE_FINISHED)
    {
        motors_stop();
        LED_OFF();
        enter_state(MS_DONE);
        return;
    }

    /* ---- 3. Ceder el paso ---- */
    if (must_yield(me, &peer) && s_state != MS_PUSH)
    {
        if (s_state != MS_YIELD || !s_yield_goal_valid)
        {
            compute_yield_point(&w, me, &peer, peer_obs,
                                &s_yield_goal_col, &s_yield_goal_row);
            s_yield_goal_valid = true;
            enter_state(MS_YIELD);
        }
        LED_YIELDING();
        if (control_drive_to(me, s_yield_goal_col, s_yield_goal_row,
                             YIELD_ARRIVE_TOLERANCE_CELLS, YIELD_SPEED))
        {
            s_yield_goal_valid = false;
        }
        return;
    }
    if (s_state == MS_YIELD)
    {
        s_yield_goal_valid = false;
        enter_state(MS_SELECT);
    }

    /* Traza de mision a 2 Hz. */
    {
        static int64_t s_trace_ms = 0;
        if (now_ms() - s_trace_ms >= 500)
        {
            s_trace_ms = now_ms();

            float goal_col = 0.0f, goal_row = 0.0f;
            bool have_goal = false;

            if (s_state == MS_GO_STAGE)
            {
                goal_col = s_detouring ? s_detour_col : s_stage_col;
                goal_row = s_detouring ? s_detour_row : s_stage_row;
                have_goal = true;
            }
            else if (s_state == MS_PUSH || s_state == MS_ALIGN)
            {
                const depot_t *d = world_find_depot(&w, s_target_color);
                if (d != NULL)
                {
                    goal_col = d->col;
                    goal_row = d->row;
                    have_goal = true;
                }
            }

            if (have_goal)
            {
                const float bearing = bearing_deg(me->col, me->row,
                                                  goal_col, goal_row);
                const cube_obs_t *trace_cube =
                    world_find_cube(&w, s_target_color);
                if (trace_cube != NULL)
                {
                    ESP_LOGI(TAG,
                             "[%s] yo(%.1f,%.1f) th=%.0f | obj %s "
                             "(%.1f,%.1f) rumbo=%.0f err=%.0f dist=%.1f | "
                             "cubo(%.1f,%.1f) in=%d age=%lu desvio=%d peer=%d",
                             mission_state_name(s_state), me->col, me->row,
                             me->theta, color_name(s_target_color), goal_col,
                             goal_row, bearing, wrap180(bearing - me->theta),
                             dist_cells(me->col, me->row, goal_col, goal_row),
                             trace_cube->col, trace_cube->row,
                             (int)trace_cube->in_depot,
                             (unsigned long)trace_cube->age_ms,
                             (int)s_detouring, (int)peer.valid);
                }
            }
            else
            {
                ESP_LOGI(TAG, "[%s] yo(%.1f,%.1f) th=%.0f | sin objetivo | peer=%d",
                         mission_state_name(s_state), me->col, me->row, me->theta,
                         (int)peer.valid);
            }
        }
    }

    /* ---- 4. Maquina de estados ---- */
    switch (s_state)
    {

    case MS_WAIT_TELEMETRY:
    case MS_WAIT_START:
        enter_state(MS_SELECT);
        break;

    case MS_SELECT:
    {
        LED_RUNNING();
        motors_stop();
        progress_reset();
        s_detouring = false;

        s_target_color = select_task(&w, me, &peer, &s_target_distance);

        if (s_target_color == COLOR_NONE)
        {
            /* O ya esta todo entregado, o el otro rover se ocupa de lo que
             * queda, o las observaciones estan viejas. En todos los casos la
             * respuesta correcta es esperar y volver a mirar. */
            enter_state(MS_DONE);
            break;
        }

        ESP_LOGI(TAG,
                 "plan optimo: %d planes evaluados, makespan %.1f s, "
                 "mio %.1f s / suyo %.1f s, %d tareas mias -> empiezo por %s",
                 s_plan.plans_evaluated, s_plan.makespan_s,
                 s_plan.my_cost_s, s_plan.peer_cost_s, s_plan.length,
                 color_name(s_target_color));

        s_task_started_ms = now_ms();
        enter_state(MS_GO_STAGE);
        break;
    }

    case MS_GO_STAGE:
    {
        LED_RUNNING();

        const cube_obs_t *cube = world_find_cube(&w, s_target_color);
        const depot_t *depot = world_find_depot(&w, s_target_color);

        if (cube == NULL || depot == NULL)
        {
            enter_state(MS_SELECT);
            break;
        }
        const cube_obs_t *corridor_blocker =
            find_push_corridor_blocker(&w, cube, depot);
        if (!stage_point_feasible(&w, cube, depot))
        {
            ESP_LOGW(TAG, "cubo %s dificil junto al borde; reintento en %d ms",
                     color_name(s_target_color), (int)EDGE_RETRY_MS);
            defer_edge_task(cube);
            enter_state(MS_SELECT);
            break;
        }
        if (edge_task_deferred(cube))
        {
            ESP_LOGI(TAG, "cubo %s aplazado temporalmente; busco otra tarea",
                     color_name(s_target_color));
            enter_state(MS_SELECT);
            break;
        }
        if (corridor_blocker != NULL)
        {
            ESP_LOGW(TAG, "corredor de %s bloqueado por %s; busco otra tarea",
                     color_name(s_target_color),
                     color_name(corridor_blocker->color));
            enter_state(MS_SELECT);
            break;
        }

        if (now_ms() - s_task_started_ms > TASK_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "tarea agotada por tiempo, replanificando");
            defer_failed_task(cube);
            enter_state(MS_SELECT);
            break;
        }

        /* --- Replanificacion continua ---
         * El mundo cambia mientras nos movemos: el otro rover entrega un cubo,
         * un cubo rueda, la camara recupera un objeto que estaba tapado. Por
         * eso el plan se recalcula en cada ciclo. Pero solo CAMBIAMOS de
         * objetivo si la mejora supera REPLAN_HYSTERESIS_S; si no, el rover
         * oscilaria entre dos opciones casi iguales sin avanzar en ninguna. */
        if (planner_solve(&w, ROVER_ID, PEER_ID, &s_plan) && s_plan.length > 0)
        {
            const cube_color_t suggested = s_plan.sequence[0];
            if (suggested != s_target_color)
            {
                const float cost_now = current_task_cost(&w, me);
                const cube_obs_t *alt = world_find_cube(&w, suggested);
                const depot_t *alt_depot = world_find_depot(&w, suggested);
                if (alt != NULL && alt_depot != NULL &&
                    !edge_task_deferred(alt) &&
                    task_feasible(&w, alt, alt_depot))
                {
                    const float cost_alt = planner_task_cost(
                        &w, me->col, me->row, me->theta, alt, NULL, NULL, NULL);
                    if (cost_alt < cost_now - REPLAN_HYSTERESIS_S)
                    {
                        ESP_LOGI(TAG, "replanifico: %s (%.1fs) -> %s (%.1fs)",
                                 color_name(s_target_color), cost_now,
                                 color_name(suggested), cost_alt);
                        s_target_color = suggested;
                        s_task_started_ms = now_ms();
                        progress_reset();
                        s_detouring = false;
                    }
                }
            }
        }

        if (obstacle_too_close() &&
            !target_cube_explains_obstacle(me, cube))
        {
            motors_stop();
            break;
        }

        compute_stage_point(cube, depot, &s_stage_col, &s_stage_row);
        clamp_to_field(&w, &s_stage_col, &s_stage_row);

        const cube_obs_t *blocking = find_blocking_cube(
            &w, me, s_stage_col, s_stage_row);
        if (s_detouring)
        {
            const cube_obs_t *old_blocker =
                world_find_cube(&w, s_detour_blocker_color);
            if (old_blocker == NULL ||
                old_blocker->age_ms > OBSERVATION_STALE_MS ||
                dist_cells(old_blocker->col, old_blocker->row,
                           s_detour_blocker_col, s_detour_blocker_row) >
                    ALIGN_CUBE_MOVE_CELLS)
            {
                s_detouring = false;
            }
        }
        if (!s_detouring && blocking != NULL)
        {
            const depot_t *blocking_depot =
                world_find_depot(&w, blocking->color);
            if (blocking_depot == NULL)
            {
                motors_stop();
                break;
            }
            compute_detour_point(&w, me, blocking, blocking_depot,
                                 s_stage_col, s_stage_row,
                                 &s_detour_col, &s_detour_row);
            s_detouring = true;
            s_detour_blocker_color = blocking->color;
            s_detour_blocker_col = blocking->col;
            s_detour_blocker_row = blocking->row;
            ESP_LOGI(TAG, "ruta bloqueada por %s: desvio fijo (%.1f, %.1f)",
                     color_name(blocking->color), s_detour_col, s_detour_row);
        }

        if (s_detouring)
        {
            if (control_drive_to(me, s_detour_col, s_detour_row,
                                 ARRIVE_TOLERANCE_CELLS, DRIVE_SPEED))
            {
                ESP_LOGI(TAG, "desvio alcanzado; voy al punto de empuje");
                s_detouring = false;
                control_reset();
            }
            break;
        }

        if (control_drive_to(me, s_stage_col, s_stage_row,
                             STAGE_ARRIVE_TOLERANCE_CELLS, DRIVE_SPEED))
        {
            s_align_cube_col = cube->col;
            s_align_cube_row = cube->row;
            enter_state(MS_ALIGN);
        }
        break;
    }

    case MS_ALIGN:
    {
        LED_PUSHING();

        const cube_obs_t *cube = world_find_cube(&w, s_target_color);
        const depot_t *depot = world_find_depot(&w, s_target_color);

        if (cube == NULL || depot == NULL)
        {
            enter_state(MS_SELECT);
            break;
        }

        /* control_drive_to ya ordeno STOP, pero la parada segura de esta placa
         * es rueda libre. Esperar antes de girar evita convertir la inercia de
         * avance en un arco que golpee el cubo. */
        if (time_in_state() < ALIGN_ENTRY_SETTLE_MS)
        {
            motors_stop();
            break;
        }

        /* Si el giro realmente movio el cubo, no se inicia un empuje diagonal.
         * Se compara el cubo directamente: la posicion del marcador del rover
         * puede describir un arco durante el giro aunque no toque el cubo. */
        if (dist_cells(cube->col, cube->row,
                       s_align_cube_col, s_align_cube_row) >
            ALIGN_CUBE_MOVE_CELLS)
        {
            ESP_LOGW(TAG, "cubo se movio durante ALIGN; recolocando");
            enter_state(MS_GO_STAGE);
            break;
        }

        /* Mirar exactamente sobre el eje cubo->deposito. Usar me->deposito
         * dejaba de atravesar el cubo si este se desplazaba durante el giro. */
        const float target_theta = bearing_deg(cube->col, cube->row,
                                               depot->col, depot->row);

        if (control_face_heading(me, target_theta))
        {
            progress_reset();
            enter_state(MS_PUSH);
        }
        else if (time_in_state() > 6000)
        {
            ESP_LOGW(TAG, "alineacion agotada; recolocando sin empujar");
            motors_stop();
            enter_state(MS_GO_STAGE);
        }
        break;
    }

    case MS_PUSH:
    {
        LED_PUSHING();

        const cube_obs_t *cube = world_find_cube(&w, s_target_color);
        const depot_t *depot = world_find_depot(&w, s_target_color);

        if (cube == NULL || depot == NULL)
        {
            motors_stop();
            enter_state(MS_SELECT);
            break;
        }

        const cube_obs_t *push_blocker =
            find_push_corridor_blocker(&w, cube, depot);
        if (push_blocker != NULL)
        {
            ESP_LOGW(TAG, "detengo %s: %s entro al corredor de empuje",
                     color_name(s_target_color),
                     color_name(push_blocker->color));
            motors_stop();
            enter_state(MS_SELECT);
            break;
        }

        /* No acelerar hacia el cubo mientras aun queda inercia del giro. */
        if (time_in_state() < PUSH_ENTRY_SETTLE_MS)
        {
            motors_stop();
            break;
        }

        if (world_cube_delivered(&w, cube))
        {
            motors_stop();
            enter_state(MS_VERIFY);
            break;
        }

        if (must_yield_during_push(me, &peer))
        {
            motors_stop();
            progress_reset();
            break;
        }

        /* Un parpadeo corto no debe producir avance a tirones, pero tampoco se
         * empuja a ciegas: tras un segundo se verifica antes de acercarse al
         * borde. Es preferible reintentar a expulsar el cubo de la cancha. */
        if (cube->age_ms > PUSH_CUBE_STALE_STOP_MS)
        {
            ESP_LOGI(TAG, "cubo ocluido durante empuje; detengo para verificar");
            motors_stop();
            enter_state(MS_VERIFY);
            break;
        }

        if (now_ms() - s_task_started_ms > TASK_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "empuje agotado por tiempo, replanificando");
            motors_stop();
            defer_failed_task(cube);
            enter_state(MS_SELECT);
            break;
        }

        /* Si el cubo se escapo de lado, volver a colocarse detras. */
        const float cube_distance =
            dist_cells(me->col, me->row, cube->col, cube->row);
        if (cube_distance > CUBE_CAPTURED_CELLS + STAGE_DISTANCE_CELLS)
        {
            ESP_LOGI(TAG, "cubo perdido durante el empuje, recolocando");
            enter_state(MS_GO_STAGE);
            break;
        }

        /* Verificar progreso solo después del corte por observación obsoleta. */
        if (push_stalled(cube, depot))
        {
            motors_stop();
            progress_reset();
            defer_failed_task(cube);
            enter_state(MS_SELECT);
            break;
        }

        /* Confirmar que el cubo esta delante y centrado en el eje de empuje. */
        const float axis_col = depot->col - cube->col;
        const float axis_row = depot->row - cube->row;
        const float axis_len = sqrtf(axis_col * axis_col + axis_row * axis_row);
        if (axis_len < 0.001f)
        {
            motors_stop();
            enter_state(MS_VERIFY);
            break;
        }

        const float ux = axis_col / axis_len;
        const float uy = axis_row / axis_len;
        const float rover_to_cube_col = cube->col - me->col;
        const float rover_to_cube_row = cube->row - me->row;
        const float ahead = rover_to_cube_col * ux + rover_to_cube_row * uy;
        const float lateral_error = fabsf(rover_to_cube_col * uy -
                                          rover_to_cube_row * ux);
        if (ahead < PUSH_BEHIND_MIN_CELLS ||
            lateral_error > PUSH_LATERAL_MAX_CELLS)
        {
            ESP_LOGW(TAG,
                     "geometria de empuje perdida: delante=%.1f lateral=%.1f; "
                     "recolocando",
                     ahead, lateral_error);
            motors_stop();
            enter_state(MS_GO_STAGE);
            break;
        }

        /* El objetivo de precision es para el CENTRO DEL ROVER, no para el
         * rover encima del centro del deposito. Si el cubo permanece delante
         * a CUBE_CAPTURED_CELLS, al llegar deja su centro sobre el del depot.
         * Luego frenamos y esperamos quietos el in_depot sostenido. */
        const float push_goal_col =
            depot->col - ux * (CUBE_CAPTURED_CELLS +
                               PUSH_STOP_BEFORE_DEPOT_CELLS);
        const float push_goal_row =
            depot->row - uy * (CUBE_CAPTURED_CELLS +
                               PUSH_STOP_BEFORE_DEPOT_CELLS);
        if (control_push_to(me, push_goal_col, push_goal_row,
                            PUSH_ARRIVE_TOLERANCE_CELLS, PUSH_SPEED))
        {
            enter_state(MS_VERIFY);
        }
        break;
    }

    case MS_VERIFY:
    {
        motors_stop();
        LED_RUNNING();

        const cube_obs_t *cube = world_find_cube(&w, s_target_color);

        /* Esperamos un momento a que la camara vea la escena ya quieta antes
         * de dar la entrega por buena. */
        if (time_in_state() < VERIFY_SETTLE_MS)
        {
            break;
        }

        if (cube != NULL && world_cube_delivered(&w, cube))
        {
            ESP_LOGI(TAG, "cubo %s entregado", color_name(s_target_color));
            if (s_target_color > COLOR_NONE && s_target_color < COLOR_COUNT)
            {
                s_task_failures[s_target_color] = 0;
                s_deferred_until_ms[s_target_color] = 0;
            }
            s_target_color = COLOR_NONE;
            enter_state(MS_SELECT);
        }
        else
        {
            ESP_LOGW(TAG, "el cubo %s no quedo dentro, reintentando",
                     color_name(s_target_color));
            s_task_started_ms = now_ms();
            enter_state(MS_GO_STAGE);
        }
        break;
    }

    case MS_DONE:
        motors_stop();
        LED_OFF();
        /* Reevaluar de vez en cuando: la vision puede volver a ver un cubo
         * que estaba oculto, o el otro rover puede abandonar su tarea. */
        if (time_in_state() > 1500)
        {
            enter_state(MS_SELECT);
        }
        break;

    default:
        motors_stop();
        enter_state(MS_SELECT);
        break;
    }

    /* ---- 5. Anunciar nuestro estado al otro rover ---- */
    static int64_t last_announce_ms = 0;
    if (now_ms() - last_announce_ms >= LINK_PERIOD_MS)
    {
        last_announce_ms = now_ms();
        esp_link_announce(s_target_color, s_target_distance, me->col, me->row,
                          s_state == MS_PUSH || s_state == MS_ALIGN);
    }
}

static void mission_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        mission_step();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MISSION_PERIOD_MS));
    }
}

void mission_start(void)
{
    s_state_entered_ms = now_ms();
    xTaskCreatePinnedToCore(mission_task, "mission", 6144, NULL, 6, NULL, 1);
}
