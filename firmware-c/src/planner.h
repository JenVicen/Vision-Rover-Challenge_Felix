/*
 * planner.h - Asignacion y secuenciacion multiagente, resuelta de forma EXACTA.
 *
 * -------------------------------------------------------------------------
 * EL PROBLEMA
 * -------------------------------------------------------------------------
 * Decidir "que rover atiende que cubo, en que orden" es una instancia de
 * Multi-Agent Task Allocation with Sequencing. En el caso general es NP-hard:
 * es un Multiple Travelling Salesman con tiempos de servicio, y minimizar el
 * makespan lo emparenta ademas con scheduling en maquinas paralelas.
 *
 * -------------------------------------------------------------------------
 * POR QUE AQUI SE PUEDE RESOLVER EXACTO
 * -------------------------------------------------------------------------
 * NP-hard es una afirmacion sobre el crecimiento asintotico, no sobre una
 * instancia concreta. Aqui la instancia esta acotada por el reglamento:
 * 2 rovers y como mucho MAX_CUBES cubes. El numero de planes completos es
 *
 *     sum_k C(n,k) * k! * (n-k)!  =  (n+1)!        (n = cubos pendientes)
 *
 * n=3 -> 24 planes, n=4 -> 120 planes. Enumerarlos todos y quedarse con el
 * mejor cuesta microsegundos en un ESP32. No hace falta heuristica: se
 * calcula el OPTIMO del modelo, no una aproximacion.
 *
 * Esto es exactamente la distincion que plantea el reto: encontrar la
 * solucion es duro en general, pero verificar un plan (evaluar su coste) es
 * barato -- y cuando el espacio de soluciones es pequeno, "verificar todos"
 * ES el algoritmo optimo.
 *
 * -------------------------------------------------------------------------
 * QUE SE OPTIMIZA
 * -------------------------------------------------------------------------
 * MAKESPAN: el tiempo del rover que termina mas tarde, no la suma. La ronda
 * acaba cuando acaba el ultimo cubo, asi que minimizar la suma premiaria
 * planes donde un rover hace todo y el otro mira. Desempate secundario por
 * suma total, para no dejar movimiento inutil.
 *
 * -------------------------------------------------------------------------
 * POR QUE NO HACE FALTA NEGOCIAR
 * -------------------------------------------------------------------------
 * Los dos rovers reciben la MISMA telemetria global (el contrato de vision es
 * un broadcast identico a ambos) y ejecutan ESTE MISMO codigo determinista,
 * con los rovers ordenados por ID. Por construccion llegan al mismo plan
 * optimo, y cada uno se queda con su parte. ESP-NOW no se usa para repartir
 * trabajo, solo para resolver colisiones fisicas y confirmar compromisos, que
 * es informacion que la camara no puede dar a tiempo.
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
