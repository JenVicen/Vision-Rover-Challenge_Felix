/*
 * world.h - Modelo del mundo, calcado del contrato oficial de vision
 *           (vision-system/contrato/CONTRATO.md, protocolo v = 2).
 *
 * Todas las posiciones estan en CELDAS (1 celda = grid.cell_mm = 20 mm).
 * col crece a la derecha, row crece hacia abajo.
 * theta en grados, 0 = +col, antihorario positivo, rango [0, 360).
 */
#ifndef WORLD_H
#define WORLD_H

#include <stdbool.h>
#include <stdint.h>

#define MAX_ROVERS 4
#define MAX_CUBES 4
#define MAX_OBSTACLES 8
#define MAX_DEPOTS 4

typedef enum
{
    PHASE_UNKNOWN = 0,
    PHASE_IDLE,
    PHASE_READY,
    PHASE_RUNNING,
    PHASE_FINISHED
} phase_t;

/* El color es la identidad del cubo: no hay dos cubos del mismo color. */
typedef enum
{
    COLOR_NONE = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_COUNT
} cube_color_t;

typedef struct
{
    int id; /* ID del marcador ArUco: 10 u 11 */
    float col;
    float row;
    float theta;     /* grados, [0, 360) */
    uint32_t age_ms; /* ms desde la ultima observacion real */
    bool present;
} rover_obs_t;

typedef struct
{
    cube_color_t color;
    float col;
    float row;
    uint32_t age_ms;
    bool present;
} cube_obs_t;

typedef struct
{
    float col;
    float row;
    uint32_t age_ms;
    bool present;
} obstacle_obs_t;

typedef struct
{
    cube_color_t color;
    float col;
    float row; /* centro de la zona de acopio */
    bool present;
} depot_t;

typedef struct
{
    /* Cabecera del mensaje */
    int protocol_version;
    uint32_t seq;
    int64_t ts_ms;
    phase_t phase;

    /* Reloj de la ronda */
    int32_t clock_elapsed_ms;
    int32_t clock_remaining_ms;
    int32_t clock_total_ms;

    /* Campo de juego */
    int grid_cols;
    int grid_rows;
    float cell_mm;

    /* Entidades */
    rover_obs_t rovers[MAX_ROVERS];
    int rover_count;
    cube_obs_t cubes[MAX_CUBES];
    int cube_count;
    obstacle_obs_t obstacles[MAX_OBSTACLES];
    int obstacle_count;
    depot_t depots[MAX_DEPOTS];
    int depot_count;

    float start_col;
    float start_row;

    float depot_length; /* lado paralelo al borde, en celdas */
    float depot_depth;  /* lado hacia adentro, en celdas */
    float cube_side;    /* arista del cubo, en celdas */

    /* Metadatos locales (no vienen del contrato) */
    int64_t rx_time_ms; /* esp_timer local al recibir el mensaje */
    bool valid;         /* true si se ha recibido al menos un mensaje bueno */
} world_t;

/* --- Utilidades de busqueda (no bloquean, operan sobre una copia) --- */

const rover_obs_t *world_find_rover(const world_t *w, int id);
const cube_obs_t *world_find_cube(const world_t *w, cube_color_t color);
const depot_t *world_find_depot(const world_t *w, cube_color_t color);

const char *color_name(cube_color_t c);
cube_color_t color_from_name(const char *name);
const char *phase_name(phase_t p);

/* true si el cubo esta completamente dentro de su deposito. */
bool world_cube_delivered(const world_t *w, const cube_obs_t *cube);

#endif /* WORLD_H */
