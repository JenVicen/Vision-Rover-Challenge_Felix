/*
 * mission.h - Logica autonoma de la ronda.
 *
 * TODA la decision ocurre aqui, dentro del rover: seleccion de cubo, reparto
 * de trabajo con el otro rover, planificacion del empuje y verificacion de la
 * entrega. Ninguna computadora externa participa (reglamento, secciones 4 y 6).
 */
#ifndef MISSION_H
#define MISSION_H

typedef enum
{
    MS_WAIT_TELEMETRY = 0, /* sin datos de la camara */
    MS_WAIT_START,         /* telemetria viva, esperando phase = RUNNING */
    MS_SELECT,             /* eligiendo a que cubo ir */
    MS_GO_STAGE,           /* yendo al punto de empuje, detras del cubo */
    MS_ALIGN,              /* orientandose hacia el deposito */
    MS_PUSH,               /* empujando el cubo */
    MS_VERIFY,             /* comprobando si el cubo quedo dentro */
    MS_YIELD,              /* cediendo el paso al otro rover */
    MS_DONE                /* no queda nada por hacer */
} mission_state_t;

void mission_start(void);

const char *mission_state_name(mission_state_t state);

#endif /* MISSION_H */
