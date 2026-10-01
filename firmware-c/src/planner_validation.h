/*
 * planner_validation.h - Diagnóstico y validación de planes para D=1.
 *
 * A dificultad D=1 máxima:
 * - Distancia media de cubos a depósitos: ~42 celdas (lejos)
 * - Interferencias: hasta 3 trayectorias pueden cruzarse (máximo conflicto)
 * - Margen para error: mínimo
 *
 * Este módulo valida que el planner produce un plan VÁLIDO y registra
 * métricas para diagnóstico cuando algo falla. No es crítico para ejecución,
 * pero es invaluable para depuración.
 */

#ifndef PLANNER_VALIDATION_H
#define PLANNER_VALIDATION_H

#include "planner.h"
#include "world.h"
#include "geom.h"

#include "esp_log.h"

#include <math.h>

static const char *TAG_VAL = "planner_val";

typedef struct
{
    /* Diagnostico del escenario. */
    int cube_count;
    float avg_distance_cells;
    int interference_count; /* 0-3: cuantas parejas de trayectorias se cruzan */

    /* Diagnostico del plan. */
    float my_cost_s;
    float peer_cost_s;
    float makespan_s;
    int plans_evaluated;

    /* Alertas. */
    bool is_high_difficulty; /* D >= 0.7 ? */
    bool makespan_tight;     /* makespan > 7 min ? (quedan solo 3) */
    bool peer_unavailable;   /* otro rover no visto */
    bool solo_plan;          /* no se pudo coordinar */

} plan_validity_t;

static inline bool planner_validate_result(const world_t *w,
                                           const plan_t *plan,
                                           plan_validity_t *out_diag)
{
    if (out_diag == NULL)
    {
        return plan->valid;
    }

    out_diag->my_cost_s = plan->my_cost_s;
    out_diag->peer_cost_s = plan->peer_cost_s;
    out_diag->makespan_s = plan->makespan_s;
    out_diag->plans_evaluated = plan->plans_evaluated;
    out_diag->solo_plan = (plan->peer_cost_s == 0.0f && plan->my_cost_s > 0.0f);

    /* Contador de cubos pendientes. */
    int cube_count = 0;
    float sum_distance = 0.0f;

    for (int i = 0; i < w->cube_count; ++i)
    {
        const cube_obs_t *c = &w->cubes[i];
        if (!c->present || c->color == COLOR_NONE || c->age_ms > OBSERVATION_STALE_MS)
            continue;
        if (world_cube_delivered(w, c))
            continue;

        cube_count++;

        const depot_t *dep = world_find_depot(w, c->color);
        if (dep != NULL)
        {
            const float dist =
                dist_cells(c->col, c->row, dep->col, dep->row);
            sum_distance += dist;
        }
    }

    out_diag->cube_count = cube_count;
    out_diag->avg_distance_cells =
        (cube_count > 0) ? (sum_distance / cube_count) : 0.0f;

    /* Estimacion simple de interferencias: si dos cubos y sus depositos
     * estan "cruzados" en el tablero, hay riesgo. No es la implementacion
     * real del generador, pero sirve para diagnostico. */
    out_diag->interference_count = 0;
    if (cube_count >= 2)
    {
        for (int i = 0; i < w->cube_count; ++i)
        {
            const cube_obs_t *c1 = &w->cubes[i];
            if (!c1->present || c1->color == COLOR_NONE ||
                c1->age_ms > OBSERVATION_STALE_MS ||
                world_cube_delivered(w, c1))
                continue;

            for (int j = i + 1; j < w->cube_count; ++j)
            {
                const cube_obs_t *c2 = &w->cubes[j];
                if (!c2->present || c2->color == COLOR_NONE ||
                    c2->age_ms > OBSERVATION_STALE_MS ||
                    world_cube_delivered(w, c2))
                    continue;

                const depot_t *d1 = world_find_depot(w, c1->color);
                const depot_t *d2 = world_find_depot(w, c2->color);
                if (d1 == NULL || d2 == NULL)
                    continue;

                /* Criterio simplificado: distancia minima entre trayectorias. */
                const float x1a = c1->col, y1a = c1->row;
                const float x1b = d1->col, y1b = d1->row;
                const float x2a = c2->col, y2a = c2->row;
                const float x2b = d2->col, y2b = d2->row;

                /* Manhattan + un margen conservador. */
                const float cross_dist =
                    (fabsf(x1a - x2a) + fabsf(y1a - y2a)) +
                    (fabsf(x1b - x2b) + fabsf(y1b - y2b));

                if (cross_dist < 32.0f) /* ~4 celdas de margen */
                {
                    out_diag->interference_count++;
                }
            }
        }
    }

    /* Alertas. */
    out_diag->is_high_difficulty = (out_diag->avg_distance_cells >= 35.0f);
    out_diag->makespan_tight = (plan->makespan_s > 420.0f); /* 7 min */
    out_diag->peer_unavailable = (plan->peer_cost_s == 0.0f);

    /* Log de diagnóstico. */
    if (!plan->valid)
    {
        ESP_LOGI(TAG_VAL, "PLAN INVÁLIDO: peers=%d/%d, cubos=%d",
                 world_find_rover(w, 10) ? 1 : 0,
                 world_find_rover(w, 11) ? 1 : 0, cube_count);
    }
    else if (out_diag->is_high_difficulty)
    {
        ESP_LOGI(TAG_VAL, "D≈1 DETECTADO: dist_avg=%.1f, interference=%d, "
                          "makespan=%.1f s, evaluated=%d",
                 out_diag->avg_distance_cells, out_diag->interference_count,
                 plan->makespan_s, plan->plans_evaluated);
    }

    return plan->valid;
}

#endif /* PLANNER_VALIDATION_H */
