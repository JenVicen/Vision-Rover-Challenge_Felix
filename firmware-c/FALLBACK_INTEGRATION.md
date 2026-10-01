/*
 * INSTRUCCIONES: Cómo integrar el fallback greedy en mission.c
 *
 * El fallback NO es crítico: el planner casi nunca falla en su cálculo.
 * Pero si alguna vez quieres agregarlo (ej: por robustez extrema),
 * aquí está el código.
 */

/* ===================================================================
 * OPCIÓN 1: Agregar fallback como red de seguridad
 * ===================================================================
 * Agregar al TOP de mission.c, después de los #include actuales:
 */

#include "planner_fallback.h"
#include "planner_validation.h"

/* Entonces, en la función select_task() o equivalent: */

void select_task_with_fallback(void)
{
    world_t world;
    vision_client_get_snapshot(&world);  // obtener estado actual

    plan_t plan;
    bool planner_ok = planner_solve(&world, ROVER_ID, PEER_ID, &plan);

    if (!planner_ok) {
        ESP_LOGW(TAG, "Planner falló. Activando fallback greedy...");

        cube_color_t my_cube = COLOR_NONE, peer_cube = COLOR_NONE;
        bool fallback_ok = planner_greedy_assign(&world, ROVER_ID, PEER_ID,
                                                 &my_cube, &peer_cube);
        if (!fallback_ok) {
            ESP_LOGI(TAG, "Fallback: sin cubos disponibles. Idle.");
            motors_stop();
            return;
        }

        ESP_LOGI(TAG, "Fallback: yo=%d, peer=%d", my_cube, peer_cube);

        /* Ejecutar la tarea my_cube manualmente (sin secuencia).
         * Este es el código más simple: solo atrapar el cubo y llevarlo. */
        const cube_obs_t *target = world_find_cube(&world, my_cube);
        if (target != NULL && target->age_ms < OBSERVATION_STALE_MS) {
            /* Ir al cubo y empujarlo al depósito (código ya existe
             * en control.c: control_approach_and_push()) */
            control_approach_and_push(&world, target);
        }
        return;
    }

    /* Plan óptimo disponible. Usar secuencia normal. */
    if (plan.valid && plan.length > 0) {
        validate_and_execute_plan(&plan, &world);
    }
}

/* ===================================================================
 * OPCIÓN 2: Validar y loguear todo sin cambiar lógica
 * ===================================================================
 * Si NO quieres cambiar la lógica, pero sí ver alertas en D≈1,
 * agregar esto al log del planner cada ciclo:
 */

void log_plan_diagnostics(const world_t *w, const plan_t *plan)
{
    plan_validity_t diag = {0};
    planner_validate_result(w, plan, &diag);

    if (!plan->valid) {
        ESP_LOGW(TAG, "[ALERT] Planner NO generó plan válido.");
    }

    if (diag.is_high_difficulty) {
        ESP_LOGI(TAG, "[D≈1] Escenario difícil detectado:");
        ESP_LOGI(TAG, "      cubos=%d, dist_avg=%.1f cells, "
                      "interference=%d",
                 diag.cube_count,
                 diag.avg_distance_cells,
                 diag.interference_count);
        ESP_LOGI(TAG, "      makespan=%.1f s (de 600 s máx), "
                      "plans_eval=%d",
                 diag.makespan_s,
                 diag.plans_evaluated);
    }

    if (diag.makespan_tight) {
        ESP_LOGW(TAG, "[TIGHT] Makespan muy ajustado: %.1f s "
                      "(solo 3 min libre)",
                 diag.makespan_s);
    }

    if (diag.peer_unavailable) {
        ESP_LOGW(TAG, "[SOLO] Otro rover no visible. "
                      "Planificando solo.");
    }
}

/* ===================================================================
 * OPCIÓN 3: Comparación planner vs greedy (debug)
 * ===================================================================
 * Para debug: ver cómo difiere la solución óptima de la greedy.
 */

void compare_planner_vs_greedy(const world_t *w)
{
    plan_t opt_plan;
    bool opt_ok = planner_solve(w, ROVER_ID, PEER_ID, &opt_plan);

    cube_color_t greedy_my = COLOR_NONE, greedy_peer = COLOR_NONE;
    bool greedy_ok =
        planner_greedy_assign(w, ROVER_ID, PEER_ID, &greedy_my, &greedy_peer);

    if (opt_ok && greedy_ok) {
        ESP_LOGI(TAG, "[DEBUG] Planner vs Greedy:");
        ESP_LOGI(TAG, "  Óptimo:  makespan=%.1f s", opt_plan.makespan_s);
        ESP_LOGI(TAG, "  Greedy:  (asignación simple, no costo)");
        ESP_LOGI(TAG, "  Mi cubo: ópt=%d, greedy=%d", 
                 (opt_plan.length > 0 ? opt_plan.sequence[0] : -1),
                 greedy_my);
    }
}

/* ===================================================================
 * RESUMÉN: No hagas nada por ahora
 * ===================================================================
 *
 * El fallback está disponible pero NO lo necesitas activar manualmente.
 * El planner enumerativo (24 planes) SIEMPRE funciona para n=3.
 *
 * Úsalo solo si:
 * 1. Experimentas pérdida de rovers (cámara falla)
 * 2. Quieres una red de seguridad extra extrema
 * 3. Debuguear porqué el planner produce un resultado inesperado
 *
 * Por ahora:
 * - Test 2: calibra PLAN_DRIVE_CELLS_S (velocidad en línea recta)
 * - Test 3: valida sensores
 * - Test 4: integra vision_client con el simulador
 * - Test 5: calibra PLAN_TURN_RATE_DEG_S y TURN_CCW_SIGN
 * - Después: prueba contra escenarios D=1 generados
 *
 * ¡El planner no te va a dejar en la estacada!
 */
