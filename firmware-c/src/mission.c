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

/* Punto de empuje calculado al entrar en MS_GO_STAGE. Se recalcula cada ciclo
 * porque el cubo se mueve mientras lo empujamos. */
static float s_stage_col = 0.0f;
static float s_stage_row = 0.0f;

/* Ultimo plan optimo calculado, para diagnostico y para la histeresis. */
static plan_t s_plan;

/* --- Lazo de verificacion de ejecucion ---
 * No basta con mandar al rover a empujar: hay que comprobar que el cubo se
 * esta acercando de verdad a su deposito. Estas variables miden ese progreso
 * real observado por la camara. */
static int64_t s_progress_last_ms = 0;
static float s_progress_last_dist = 1e6f;
static int s_progress_fails = 0;

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

/*
 * Si la recta hacia el punto de empuje pasa por encima del cubo, ir directo
 * lo desplazaria antes de tiempo. En ese caso se inserta un desvio lateral.
 */
static bool path_crosses_cube(const rover_obs_t *me, float goal_col, float goal_row,
                              const cube_obs_t *cube)
{
    const float vx = goal_col - me->col;
    const float vy = goal_row - me->row;
    const float len_sq = vx * vx + vy * vy;

    if (len_sq < 0.001f)
    {
        return false;
    }

    /* Proyeccion escalar del cubo sobre el segmento, acotada a [0,1]. */
    float t = ((cube->col - me->col) * vx + (cube->row - me->row) * vy) / len_sq;
    t = clampf(t, 0.0f, 1.0f);

    const float closest_col = me->col + t * vx;
    const float closest_row = me->row + t * vy;

    const float clearance = ROVER_RADIUS_CELLS + 1.0f;
    return dist_cells(closest_col, closest_row, cube->col, cube->row) < clearance;
}

static void detour_point(const rover_obs_t *me, const cube_obs_t *cube,
                         float *out_col, float *out_row)
{
    /* Rodear el cubo por el lado que ya nos queda mas cerca. */
    const float dc = cube->col - me->col;
    const float dr = cube->row - me->row;
    const float len = sqrtf(dc * dc + dr * dr);

    if (len < 0.001f)
    {
        *out_col = me->col;
        *out_row = me->row;
        return;
    }

    /* Perpendicular unitaria a la direccion rover->cubo. */
    const float px = -dr / len;
    const float py = dc / len;

    const float offset = ROVER_RADIUS_CELLS * 2.5f;

    *out_col = cube->col + px * offset;
    *out_row = cube->row + py * offset;
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
 * Verificacion de ejecucion
 * --------------------------------------------------------------------
 * "Una trayectoria correcta en simulacion puede fallar al ejecutarse."
 * Por eso no damos por bueno el empuje: medimos, con la camara, si la
 * distancia cubo->deposito realmente esta bajando. Si no baja, el plan es
 * irrelevante -- lo que hay es un fallo fisico, y toca recolocarse.
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

    const peer_status_t peer = esp_link_peer();

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
        motors_stop();
        LED_YIELDING();
        enter_state(MS_YIELD);
        /* Se reevalua en el siguiente ciclo; en cuanto el otro se aleje o
         * deje de tener prioridad, volvemos a MS_SELECT. */
        if (time_in_state() > 3000)
        {
            /* Si el bloqueo se eterniza, replanificamos: quiza convenga
             * atender otro cubo en vez de esperar. */
            enter_state(MS_SELECT);
        }
        return;
    }
    if (s_state == MS_YIELD)
    {
        enter_state(MS_SELECT);
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

        if (now_ms() - s_task_started_ms > TASK_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "tarea agotada por tiempo, replanificando");
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
                if (alt != NULL)
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
                    }
                }
            }
        }

        if (obstacle_too_close())
        {
            motors_stop();
            break;
        }

        compute_stage_point(cube, depot, &s_stage_col, &s_stage_row);
        clamp_to_field(&w, &s_stage_col, &s_stage_row);

        float goal_col = s_stage_col;
        float goal_row = s_stage_row;

        if (path_crosses_cube(me, goal_col, goal_row, cube))
        {
            detour_point(me, cube, &goal_col, &goal_row);
            clamp_to_field(&w, &goal_col, &goal_row);
        }

        if (control_drive_to(me, goal_col, goal_row,
                             ARRIVE_TOLERANCE_CELLS, DRIVE_SPEED))
        {
            /* Solo pasamos a alinear si llegamos al punto de empuje real,
             * no si lo que alcanzamos era el desvio. */
            const float to_stage = dist_cells(me->col, me->row,
                                              s_stage_col, s_stage_row);
            if (to_stage <= ARRIVE_TOLERANCE_CELLS * 1.5f)
            {
                enter_state(MS_ALIGN);
            }
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

        /* Mirar al deposito atravesando el cubo. */
        const float target_theta = bearing_deg(me->col, me->row,
                                               depot->col, depot->row);

        if (control_face_heading(me, target_theta) || time_in_state() > 6000)
        {
            progress_reset();
            enter_state(MS_PUSH);
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

        if (world_cube_delivered(&w, cube))
        {
            motors_stop();
            enter_state(MS_VERIFY);
            break;
        }

        if (now_ms() - s_task_started_ms > TASK_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "empuje agotado por tiempo, replanificando");
            motors_stop();
            enter_state(MS_SELECT);
            break;
        }

        /* Si el cubo se escapo de lado, volver a colocarse detras. */
        const float lateral = dist_cells(me->col, me->row, cube->col, cube->row);
        if (lateral > CUBE_CAPTURED_CELLS + STAGE_DISTANCE_CELLS)
        {
            ESP_LOGI(TAG, "cubo perdido durante el empuje, recolocando");
            enter_state(MS_GO_STAGE);
            break;
        }

        /* Verificacion de ejecucion: el cubo debe estar ACERCANDOSE al
         * deposito. Si no lo hace, da igual lo bueno que fuera el plan:
         * hay un fallo fisico y hay que recolocarse y volver a decidir. */
        if (push_stalled(cube, depot))
        {
            motors_stop();
            progress_reset();
            enter_state(MS_GO_STAGE);
            break;
        }

        /* Se apunta al centro del deposito y se atraviesa sin frenar. */
        control_push_through(me, depot->col, depot->row,
                             ARRIVE_TOLERANCE_CELLS, PUSH_SPEED);
        break;
    }

    case MS_VERIFY:
    {
        motors_stop();
        LED_RUNNING();

        const cube_obs_t *cube = world_find_cube(&w, s_target_color);

        /* Esperamos un momento a que la camara vea la escena ya quieta antes
         * de dar la entrega por buena. */
        if (time_in_state() < 400)
        {
            break;
        }

        if (cube != NULL && world_cube_delivered(&w, cube))
        {
            ESP_LOGI(TAG, "cubo %s entregado", color_name(s_target_color));
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
