/*
 * config.h - Todo lo que hay que ajustar antes de una ronda.
 *
 * Nada de esto se cambia durante la ronda: el reglamento (seccion 8) permite
 * configurar IP, MAC y parametros ANTES de iniciar, y prohibe tocarlos despues.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ===================================================================
 * 1. IDENTIDAD DEL ROVER
 * ===================================================================
 * ROVER_ID llega por -DROVER_ID=10 / 11 desde platformio.ini.
 * Debe coincidir con el ID del marcador ArUco pegado al rover.
 */
#ifndef ROVER_ID
#define ROVER_ID 10
#endif

#if ROVER_ID == 10
#define PEER_ID 11
#else
#define PEER_ID 10
#endif

/* MAC del OTRO rover, para ESP-NOW.
 * Se obtiene del log de arranque del otro rover ("MAC propia: ...").
 * Rellenar AMBOS, y compilar cada env con su pareja correcta. */
#define PEER_MAC_ROVER_10 {0x70, 0x4B, 0xCA, 0x5E, 0xBC, 0x10}
#define PEER_MAC_ROVER_11 {0x70, 0x4B, 0xCA, 0x5E, 0xBC, 0x7C}

/* ===================================================================
 * 2. RED
 * ===================================================================
 * Los dos rovers y la PC de vision deben estar en la MISMA red.
 * ESP-NOW usa el canal de la red Wi-Fi (peer channel = 0), asi que
 * no hay que configurar canal a mano.
 */
#define WIFI_SSID "Chanel 2.4G"
#define WIFI_PASSWORD "2704012168"

/* IP de la computadora que corre el sistema oficial de vision. */
#define VISION_HOST "192.168.1.219"

/* Puerto del contrato oficial (schema.py: DEFAULT_PORT = 2026). */
#define VISION_PORT 2026

/* ===================================================================
 * 3. GEOMETRIA DEL CAMPO (contrato de vision)
 * ===================================================================
 * Unidades: "celdas". 1 celda = 20 mm (grid.cell_mm).
 * col crece a la derecha, row crece hacia abajo.
 * theta en grados, 0 = +col, antihorario positivo, rango [0,360).
 */
#define CELL_MM 20.0f

/* Medio ancho del rover en celdas, usado para separaciones de seguridad.
 * El CenfoBot mide ~95 mm de ancho -> ~4.75 celdas -> radio ~2.5 celdas. */
#define ROVER_RADIUS_CELLS 3.0f

/* ===================================================================
 * 4. CALIBRACION DE MOTORES
 * ===================================================================
 * MOTOR_RIGHT_GAIN: salida del motor 2 multiplicada por este factor para
 * compensar la diferencia mecanica entre lados. Se obtiene con la prueba
 * de linea recta descrita en el README (equivale a motor_calibration.py).
 * Rover 11 medido con desvio positivo: bajar ~0.03 para compensar exceso del
 * motor derecho.
 */
#define MOTOR_RIGHT_GAIN 0.97f

/* Invertir el sentido de un motor si quedo cableado al reves.
 * Las inversiones se calibran por rover porque el cableado puede variar. */
#if ROVER_ID == 10
#define MOTOR_LEFT_INVERT 0
#define MOTOR_RIGHT_INVERT 1
#elif ROVER_ID == 11
#define MOTOR_LEFT_INVERT 1
#define MOTOR_RIGHT_INVERT 1
#else
#error "ROVER_ID debe ser 10 o 11"
#endif

/* Signo del giro. Si con TURN_CCW_SIGN = +1 el rover gira al reves de lo
 * que dice la vision (theta disminuye cuando deberia aumentar), poner -1.
 * Se verifica con la prueba de giro del README. */
#define TURN_CCW_SIGN (+1)

/* Minimo throttle que realmente mueve al robot (zona muerta del puente H).
 * Por debajo de esto los motores zumban pero no giran.
 * Reduced to 0.05 for battery weight pressing wheels (tested Sep 20). */
#define MOTOR_DEADBAND 0.05f

/* ===================================================================
 * 5. CONTROL DE MOVIMIENTO
 * ===================================================================
 * El control de rumbo usa theta ABSOLUTO de la vision (20 Hz) como termino
 * proporcional, y el giroscopio (~100 Hz) como termino derivativo. Asi el
 * rumbo no acumula deriva y aun asi reacciona entre frames de camara.
 */
#define HEADING_KP 0.0130f /* por grado de error */
#define HEADING_KD 0.0035f /* por grado/s de velocidad angular */
#define HEADING_MAX_CORR 0.35f

#define TURN_KP 0.0090f
#define TURN_MIN_SPEED 0.20f
#define TURN_MAX_SPEED 0.45f
#define TURN_TOLERANCE_DEG 6.0f /* error angular aceptable al terminar */

#define DRIVE_SPEED 0.45f    /* crucero */
#define PUSH_SPEED 0.38f     /* empujando un cubo */
#define APPROACH_SPEED 0.30f /* ultimos centimetros */
#define ARRIVE_TOLERANCE_CELLS 1.8f
#define SLOWDOWN_RADIUS_CELLS 6.0f

/* Si el error de rumbo supera esto, se gira en sitio antes de avanzar. */
#define REALIGN_THRESHOLD_DEG 35.0f

/* ===================================================================
 * 6. ESTRATEGIA DE EMPUJE
 * ===================================================================
 * Para empujar un cubo hacia su deposito, el rover se coloca en un punto
 * "detras" del cubo sobre la recta cubo->deposito, y luego avanza.
 */
#define STAGE_DISTANCE_CELLS 6.0f /* cuan atras del cubo se posiciona */
#define CUBE_CAPTURED_CELLS 4.0f  /* distancia a la que se considera en contacto */

/* Un cubo se da por entregado cuando esta completamente dentro del deposito.
 * Margen conservador: media diagonal del cubo (cube_side * sqrt(2) / 2). */
#define DELIVERY_MARGIN_CELLS 0.3f

/* ===================================================================
 * 7. SEGURIDAD Y TIEMPOS
 * ===================================================================
 */
/* Si la telemetria es mas vieja que esto, el rover se detiene. */
#define TELEMETRY_TIMEOUT_MS 700

/* Si la observacion de un objeto es mas vieja que esto, no se confia en ella
 * para decidir (contrato: campo age_ms). */
#define OBSERVATION_STALE_MS 1200

/* Distancia a la que se considera que el otro rover estorba. */
#define PEER_AVOID_CELLS 12.0f

/* Distancia ultrasonica que dispara frenado de emergencia. */
#define ULTRA_EMERGENCY_CM 7.0f

/* Periodos de las tareas. */
#define MISSION_PERIOD_MS 20 /* 50 Hz */
#define LINK_PERIOD_MS 150

/* Tiempo maximo persiguiendo una misma tarea antes de reintentar. */
#define TASK_TIMEOUT_MS 25000

/* ===================================================================
 * 8. MODELO DE COSTE DEL PLANIFICADOR
 * ===================================================================
 * El planificador convierte geometria en SEGUNDOS para poder minimizar el
 * makespan. Estos numeros no tienen que ser exactos, pero si tienen que ser
 * RELATIVAMENTE correctos entre si: lo que decide el reparto es que girar sea
 * caro comparado con avanzar, y que empujar sea mas lento que ir en vacio.
 *
 * Se miden con la prueba 2 del README (recta) y la prueba 5 (giro):
 *   PLAN_DRIVE_CELLS_S   = celdas recorridas / segundos, a DRIVE_SPEED
 *   PLAN_TURN_RATE_DEG_S = grados girados / segundos, a TURN_MAX_SPEED
 */
#define PLAN_DRIVE_CELLS_S 1.24f   /* medido: 44 cm en 2 s a throttle 0.40 */
#define PLAN_PUSH_CELLS_S 0.80f    /* empujando, estimado ~65% del vacio */
#define PLAN_TURN_RATE_DEG_S 90.0f /* giro en sitio */
#define PLAN_SERVICE_TIME_S 2.5f   /* alinear + verificar + soltar */

/* ===================================================================
 * 9. LAZO DE VERIFICACION DE EJECUCION
 * ===================================================================
 * El plan optimo sobre el papel no sirve de nada si el empuje real no
 * progresa. Estos parametros definen cuando se declara que la ejecucion
 * fallo y hay que volver a decidir.
 */
/* Cada cuanto se comprueba si el cubo se esta acercando a su deposito. */
#define PROGRESS_CHECK_MS 1200

/* Reduccion minima de la distancia cubo->deposito en esa ventana.
 * Si en 1.2 s el cubo no se acerco al menos esto, algo va mal: el rover
 * perdio el contacto, esta patinando, o empuja en angulo. */
#define PROGRESS_MIN_CELLS 1.0f

/* Cuantos fallos de progreso consecutivos antes de abortar la tarea. */
#define PROGRESS_MAX_FAILS 2

/* Replanificamos continuamente, pero solo CAMBIAMOS de objetivo si el plan
 * nuevo mejora al actual por mas de este margen. Sin esta histeresis el
 * rover oscilaria entre dos cubos casi equivalentes y no avanzaria ninguno. */
#define REPLAN_HYSTERESIS_S 2.0f

/* Una vez que estamos empujando, no soltamos el cubo por un plan mejor:
 * el trabajo ya invertido se perderia. Solo se abandona por fallo real. */
#define COMMIT_ON_PUSH 1

#endif /* CONFIG_H */
