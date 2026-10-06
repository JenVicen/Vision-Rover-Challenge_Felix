/*
 * mission.h - Logica autonoma de la ronda.
 *
 * Ejecuta el plan local, coordina el paso entre rovers y verifica entregas.
 * Las decisiones se calculan dentro del rover; la computadora externa solo
 * publica telemetria.
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
