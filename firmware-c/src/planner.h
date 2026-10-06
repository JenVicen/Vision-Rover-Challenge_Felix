/*
 * planner.h - Asignacion y secuenciacion determinista para dos rovers.
 *
 * Se enumeran todas las asignaciones y ordenes permitidos por el limite de
 * cubos del contrato. El criterio principal es el makespan; la suma de tiempos
 * se usa como desempate. El orden canonico por ID y color mantiene el mismo
 * resultado en ambos rovers.
 */
#ifndef PLANNER_H
#define PLANNER_H

#include "world.h"

#include <stdbool.h>

#define PLANNER_MAX_TASKS MAX_CUBES

typedef struct
{
    /* Secuencia de cubos que me toca a mi, en orden. */
    cube_color_t sequence[PLANNER_MAX_TASKS];
    int length;

    /* Coste estimado, en segundos, de mi secuencia completa. */
    float my_cost_s;

    /* Coste del otro rover, para diagnostico y para el desempate de paso. */
    float peer_cost_s;

    /* Makespan del plan elegido = max(my_cost_s, peer_cost_s). */
    float makespan_s;

    /* Cuantos planes se evaluaron (util para el log y para la defensa). */
    int plans_evaluated;

    bool valid;
} plan_t;

/*
 * Calcula el plan optimo para el mundo observado.
 *
 * my_id      : ROVER_ID de este robot.
 * peer_id    : ROVER_ID del otro robot.
 * out        : plan resultante; out->sequence contiene SOLO mis tareas.
 *
 * Si el otro rover no es visible, se planifica en solitario (todas las tareas
 * pendientes son mias), que es el comportamiento seguro: mejor duplicar
 * esfuerzo que dejar un cubo sin atender.
 *
 * Devuelve false si no hay nada pendiente que yo pueda hacer.
 */
bool planner_solve(const world_t *w, int my_id, int peer_id, plan_t *out);

/*
 * Coste estimado, en segundos, de que un rover situado en (col,row) con rumbo
 * theta lleve el cubo indicado hasta su deposito. Expuesto porque la maquina
 * de estados lo usa para decidir si merece la pena abandonar la tarea actual.
 */
float planner_task_cost(const world_t *w, float col, float row, float theta,
                        const cube_obs_t *cube,
                        float *out_end_col, float *out_end_row,
                        float *out_end_theta);

#endif /* PLANNER_H */
