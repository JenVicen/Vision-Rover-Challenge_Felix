/*
 * planner_fallback.h - Algoritmo greedy como fallback si el planner falla.
 *
 * Cuando el planner optimo no puede generar un plan (ej: cámara perdió ambos
 * rovers, o timeout), este módulo ofrece una estrategia greedy funcional:
 *
 * 1. Cada rover selecciona el cubo más cercano disponible.
 * 2. Si seleccionan el mismo, asignarlo al más cercano; el otro elige siguiente.
 * 3. Evitar interferencias simples (si las trayectorias se cruzan, esperar).
 *
 * Es subóptimo pero GARANTIZA que todos los cubos se atiendan, incluso si el
 * planificador falla temporalmente. La replanificación ocurre cada 20 ms.
 */

#ifndef PLANNER_FALLBACK_H
#define PLANNER_FALLBACK_H

#include "world.h"
#include "config.h"
#include "geom.h"

#include <stddef.h>
#include <math.h>

/*
 * Solución greedy simple. Devuelve un plan válido incluso si es subóptimo.
 *
 * my_id, peer_id: identificadores de los rovers
 * w             : estado actual del mundo (visual + cubos)
 * my_cube_color : (OUT) color del cubo asignado a este rover (-1 si nada)
 * peer_cube_color : (OUT) color del cubo asignado al otro (-1 si nada)
 *
 * Retorna true si asignó al menos un cubo.
 */
bool planner_greedy_assign(const world_t *w, int my_id, int peer_id,
                           cube_color_t *my_cube_color,
                           cube_color_t *peer_cube_color)
{
    const rover_obs_t *me = world_find_rover(w, my_id);
    const rover_obs_t *peer = world_find_rover(w, peer_id);

    if (me == NULL)
    {
        return false;
    }

    /* Cubos pendientes y no stalados. */
    const cube_obs_t *pending[MAX_CUBES];
    int n = 0;

    for (int i = 0; i < w->cube_count && n < MAX_CUBES; ++i)
    {
        const cube_obs_t *c = &w->cubes[i];
        if (!c->present || c->color == COLOR_NONE)
            continue;
        if (c->age_ms > OBSERVATION_STALE_MS)
            continue;
        if (world_cube_delivered(w, c))
            continue;
        pending[n++] = c;
    }

    if (n == 0)
    {
        return false;
    }

    /* Calcula distancia de un rover a un cubo. */
    auto cube_distance = [](const rover_obs_t *r, const cube_obs_t *c) -> float
    {
        const float dc = c->col - r->col;
        const float dr = c->row - r->row;
        return sqrtf(dc * dc + dr * dr);
    };

    /* Selecciona el cubo más cercano no asignado. */
    auto nearest_cube = [&](const rover_obs_t *r, const cube_color_t *excluded,
                            int excluded_count) -> int
    {
        int best = -1;
        float best_dist = 1e9f;
        for (int i = 0; i < n; ++i)
        {
            bool is_excluded = false;
            for (int j = 0; j < excluded_count; ++j)
            {
                if (pending[i]->color == excluded[j])
                {
                    is_excluded = true;
                    break;
                }
            }
            if (is_excluded)
                continue;

            const float d = cube_distance(r, pending[i]);
            if (d < best_dist)
            {
                best_dist = d;
                best = i;
            }
        }
        return best;
    };

    /* Mi cubo más cercano. */
    int my_nearest_idx = nearest_cube(me, NULL, 0);
    if (my_nearest_idx < 0)
    {
        return false; /* No hay cubos. */
    }

    *my_cube_color = pending[my_nearest_idx]->color;

    /* Si no vemos al otro, mi cubo está asignado. */
    if (peer == NULL || peer->age_ms > OBSERVATION_STALE_MS)
    {
        *peer_cube_color = COLOR_NONE;
        return true;
    }

    /* Su cubo más cercano (excluido el mío si eligió lo mismo). */
    cube_color_t excluded[] = {*my_cube_color};
    int peer_nearest_idx = nearest_cube(peer, excluded, 1);

    if (peer_nearest_idx < 0)
    {
        /* Solo hay un cubo, o el otro solo ve el mío. Dejarle nada. */
        *peer_cube_color = COLOR_NONE;
        return true;
    }

    *peer_cube_color = pending[peer_nearest_idx]->color;
    return true;
}

#endif /* PLANNER_FALLBACK_H */
