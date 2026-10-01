#include "planner.h"

#include "config.h"
#include "geom.h"

#include "esp_log.h"

#include <math.h>
#include <string.h>

static const char *TAG = "planner";

/* --------------------------------------------------------------------
 * Modelo de coste
 * --------------------------------------------------------------------
 * Convertimos geometria en SEGUNDOS, porque el objetivo del reto es tiempo.
 * El modelo es deliberadamente simple: velocidad de crucero constante,
 * velocidad de empuje constante, y un cargo por cada grado que hay que girar.
 * Un modelo mas fino no serviria de nada -- la incertidumbre real del mundo
 * (patinaje, cubos que se desvian) es mayor que el error del modelo, y el
 * plan se recalcula 50 veces por segundo de todos modos.
 * -------------------------------------------------------------------- */

static float travel_time_s(float from_col, float from_row, float from_theta,
                           float to_col, float to_row, float speed_cells_s,
                           float *out_theta)
{
    const float d = dist_cells(from_col, from_row, to_col, to_row);

    float heading = from_theta;
    if (d > 0.01f)
    {
        heading = bearing_deg(from_col, from_row, to_col, to_row);
    }

    const float turn_deg = fabsf(wrap180(heading - from_theta));

    if (out_theta != NULL)
    {
        *out_theta = heading;
    }

    return (d / speed_cells_s) + (turn_deg / PLAN_TURN_RATE_DEG_S);
}

static void stage_point_of(const cube_obs_t *cube, const depot_t *depot,
                           float *out_col, float *out_row)
{
    const float dc = depot->col - cube->col;
    const float dr = depot->row - cube->row;
    const float len = sqrtf(dc * dc + dr * dr);

    if (len < 0.001f)
    {
        *out_col = cube->col;
        *out_row = cube->row;
        return;
    }
    *out_col = cube->col - (dc / len) * STAGE_DISTANCE_CELLS;
    *out_row = cube->row - (dr / len) * STAGE_DISTANCE_CELLS;
}

float planner_task_cost(const world_t *w, float col, float row, float theta,
                        const cube_obs_t *cube,
                        float *out_end_col, float *out_end_row,
                        float *out_end_theta)
{
    const depot_t *depot = world_find_depot(w, cube->color);
    if (depot == NULL)
    {
        return 1e6f;
    }

    float stage_col, stage_row;
    stage_point_of(cube, depot, &stage_col, &stage_row);

    float th = theta;

    /* Tramo 1: ir al punto de empuje, a velocidad de crucero. */
    float t = travel_time_s(col, row, th, stage_col, stage_row,
                            PLAN_DRIVE_CELLS_S, &th);

    /* Tramo 2: alinearse mirando al deposito a traves del cubo. */
    const float push_heading = bearing_deg(stage_col, stage_row,
                                           depot->col, depot->row);
    t += fabsf(wrap180(push_heading - th)) / PLAN_TURN_RATE_DEG_S;
    th = push_heading;

    /* Tramo 3: empujar, mas lento. */
    const float push_dist = dist_cells(stage_col, stage_row,
                                       depot->col, depot->row);
    t += push_dist / PLAN_PUSH_CELLS_S;

    /* Cargo fijo por maniobrar, verificar la entrega y desengancharse. */
    t += PLAN_SERVICE_TIME_S;

    if (out_end_col != NULL)
        *out_end_col = depot->col;
    if (out_end_row != NULL)
        *out_end_row = depot->row;
    if (out_end_theta != NULL)
        *out_end_theta = th;

    return t;
}

/* Coste de una secuencia completa de cubos para un rover. */
static float sequence_cost(const world_t *w, const rover_obs_t *start,
                           const cube_obs_t *const *cubes,
                           const int *order, int count)
{
    if (count == 0)
    {
        return 0.0f;
    }

    float col = start->col;
    float row = start->row;
    float theta = start->theta;
    float total = 0.0f;

    for (int i = 0; i < count; ++i)
    {
        total += planner_task_cost(w, col, row, theta, cubes[order[i]],
                                   &col, &row, &theta);
    }
    return total;
}

/* --------------------------------------------------------------------
 * Enumeracion exhaustiva
 * --------------------------------------------------------------------
 * Un plan queda descrito por:
 *   - una mascara de bits: que cubos van al rover A
 *   - un orden dentro de A
 *   - un orden dentro de B
 *
 * Generamos todas las permutaciones de los n cubos pendientes y, para cada
 * permutacion, todos los puntos de corte... no: eso repite trabajo. En su
 * lugar recorremos las 2^n mascaras y, para cada una, permutamos cada lado.
 * Con n <= 4 el total cabe holgadamente en el presupuesto de 20 ms por ciclo.
 * -------------------------------------------------------------------- */

/* Permutaciones en orden lexicografico por indice (algoritmo de Heap iterativo
 * seria mas rapido, pero necesitamos DETERMINISMO exacto entre los dos rovers,
 * asi que usamos un generador simple y totalmente reproducible). */
static bool next_permutation_int(int *a, int n)
{
    if (n < 2)
    {
        return false;
    }
    int i = n - 2;
    while (i >= 0 && a[i] >= a[i + 1])
    {
        --i;
    }
    if (i < 0)
    {
        return false;
    }
    int j = n - 1;
    while (a[j] <= a[i])
    {
        --j;
    }
    const int tmp = a[i];
    a[i] = a[j];
    a[j] = tmp;

    for (int l = i + 1, r = n - 1; l < r; ++l, --r)
    {
        const int t = a[l];
        a[l] = a[r];
        a[r] = t;
    }
    return true;
}

bool planner_solve(const world_t *w, int my_id, int peer_id, plan_t *out)
{
    memset(out, 0, sizeof(*out));

    /* ---- 1. Cubos pendientes (observacion fresca y no entregado) ---- */
    const cube_obs_t *pending[PLANNER_MAX_TASKS];
    int n = 0;

    for (int i = 0; i < w->cube_count && n < PLANNER_MAX_TASKS; ++i)
    {
        const cube_obs_t *c = &w->cubes[i];
        if (!c->present || c->color == COLOR_NONE)
        {
            continue;
        }
        if (c->age_ms > OBSERVATION_STALE_MS)
        {
            continue;
        }
        if (world_cube_delivered(w, c))
        {
            continue;
        }
        if (world_find_depot(w, c->color) == NULL)
        {
            continue;
        }
        pending[n++] = c;
    }

    if (n == 0)
    {
        return false;
    }

    /* Orden canonico por color: garantiza que ambos rovers enumeren los
     * planes en el mismo orden y por tanto rompan los empates igual. */
    for (int i = 1; i < n; ++i)
    {
        const cube_obs_t *key = pending[i];
        int j = i - 1;
        while (j >= 0 && pending[j]->color > key->color)
        {
            pending[j + 1] = pending[j];
            --j;
        }
        pending[j + 1] = key;
    }

    /* ---- 2. Los dos rovers, ordenados por ID (canonico) ---- */
    const rover_obs_t *me = world_find_rover(w, my_id);
    const rover_obs_t *peer = world_find_rover(w, peer_id);

    if (me == NULL || me->age_ms > OBSERVATION_STALE_MS)
    {
        return false;
    }
    if (peer != NULL && peer->age_ms > OBSERVATION_STALE_MS)
    {
        peer = NULL;
    }

    /* Si no vemos al otro, planificamos en solitario: todas las tareas mias.
     * Es la opcion segura -- duplicar esfuerzo cuesta tiempo, no puntos. */
    if (peer == NULL)
    {
        float order_cost = 1e9f;
        int best[PLANNER_MAX_TASKS];
        int ord[PLANNER_MAX_TASKS];
        int evaluated = 0;

        for (int i = 0; i < n; ++i)
        {
            ord[i] = i;
            best[i] = i;
        }
        do
        {
            const float c = sequence_cost(w, me, pending, ord, n);
            ++evaluated;
            if (c < order_cost)
            {
                order_cost = c;
                memcpy(best, ord, sizeof(int) * (size_t)n);
            }
        } while (next_permutation_int(ord, n));

        for (int i = 0; i < n; ++i)
        {
            out->sequence[i] = pending[best[i]]->color;
        }
        out->length = n;
        out->my_cost_s = order_cost;
        out->peer_cost_s = 0.0f;
        out->makespan_s = order_cost;
        out->plans_evaluated = evaluated;
        out->valid = true;
        return true;
    }

    /* A = rover con ID menor, B = el otro. Canonico e identico en ambos. */
    const bool i_am_a = (my_id < peer_id);
    const rover_obs_t *rover_a = i_am_a ? me : peer;
    const rover_obs_t *rover_b = i_am_a ? peer : me;

    float best_makespan = 1e9f;
    float best_total = 1e9f;
    int best_a[PLANNER_MAX_TASKS], best_b[PLANNER_MAX_TASKS];
    int best_na = 0, best_nb = 0;
    float best_cost_a = 0.0f, best_cost_b = 0.0f;
    int evaluated = 0;

    const int mask_count = 1 << n;

    for (int mask = 0; mask < mask_count; ++mask)
    {
        int idx_a[PLANNER_MAX_TASKS], idx_b[PLANNER_MAX_TASKS];
        int na = 0, nb = 0;

        for (int i = 0; i < n; ++i)
        {
            if (mask & (1 << i))
            {
                idx_a[na++] = i;
            }
            else
            {
                idx_b[nb++] = i;
            }
        }

        /* Permutaciones del lado A. */
        int perm_a[PLANNER_MAX_TASKS];
        memcpy(perm_a, idx_a, sizeof(int) * (size_t)(na > 0 ? na : 1));

        do
        {
            const float cost_a = sequence_cost(w, rover_a, pending, perm_a, na);

            /* Poda: si A solo ya supera el mejor makespan, ninguna
             * permutacion de B puede salvar este reparto. */
            if (cost_a >= best_makespan)
            {
                if (na < 2)
                    break;
                continue;
            }

            int perm_b[PLANNER_MAX_TASKS];
            memcpy(perm_b, idx_b, sizeof(int) * (size_t)(nb > 0 ? nb : 1));

            do
            {
                const float cost_b = sequence_cost(w, rover_b, pending, perm_b, nb);
                ++evaluated;

                const float makespan = (cost_a > cost_b) ? cost_a : cost_b;
                const float total = cost_a + cost_b;

                /* Criterio principal: makespan. Secundario: trabajo total. */
                if (makespan < best_makespan - 0.001f ||
                    (fabsf(makespan - best_makespan) <= 0.001f &&
                     total < best_total - 0.001f))
                {
                    best_makespan = makespan;
                    best_total = total;
                    best_cost_a = cost_a;
                    best_cost_b = cost_b;
                    best_na = na;
                    best_nb = nb;
                    if (na > 0)
                        memcpy(best_a, perm_a, sizeof(int) * (size_t)na);
                    if (nb > 0)
                        memcpy(best_b, perm_b, sizeof(int) * (size_t)nb);
                }

                if (nb < 2)
                    break;
            } while (next_permutation_int(perm_b, nb));

            if (na < 2)
                break;
        } while (next_permutation_int(perm_a, na));
    }

    if (best_makespan >= 1e9f)
    {
        return false;
    }

    /* ---- 3. Quedarme con MI parte del plan optimo ---- */
    const int *mine = i_am_a ? best_a : best_b;
    const int n_mine = i_am_a ? best_na : best_nb;

    for (int i = 0; i < n_mine; ++i)
    {
        out->sequence[i] = pending[mine[i]]->color;
    }
    out->length = n_mine;
    out->my_cost_s = i_am_a ? best_cost_a : best_cost_b;
    out->peer_cost_s = i_am_a ? best_cost_b : best_cost_a;
    out->makespan_s = best_makespan;
    out->plans_evaluated = evaluated;
    out->valid = true;

    ESP_LOGD(TAG, "%d cubos, %d planes, makespan %.1fs, mias %d",
             n, evaluated, best_makespan, n_mine);

    return n_mine > 0;
}
