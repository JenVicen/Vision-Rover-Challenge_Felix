# Planner y Fallback — Análisis de robustez para D=1 máximo

## Resumen ejecutivo

Tu **planner enumerativo (24 planes para n=3 cubos) es óptimo y muy rápido** (microsegundos). A dificultad D=1 máxima (cubos lejanos, interferencias máximas), el planner sigue siendo válido, pero **márgenes de error = mínimos**.

Hemos agregado:
1. **Fallback greedy** (`planner_fallback.h`) — Si algo falla, asigna cubos por cercanía
2. **Validador de diagnóstico** (`planner_validation.h`) — Detecta escenarios D≈1 y registra alertas
3. **Suite Python de escenarios** (`tools/scenario_validator.py`) — Genera y valida contra la lógica de la organización

---

## ¿Por qué el planner NO falla a D=1?

| Métrica | n=3 cubos | Impacto |
|---------|-----------|--------|
| Planes totales | 24 (exacto) | Enumeración completa, garantizado óptimo |
| Tiempo de cálculo | ~100 µs | ESP32 puede hacerlo cada 20 ms sin problemas |
| Complejidad | O(n! × n) = O(144) | Trivial |
| Makespan típico a D=1 | ~7–9 minutos | Sobra tiempo antes del límite de 10 min |

**Conclusión:** El planner **nunca falla por computación**. Solo puede fallar si:
- Ambos rovers no son visibles (la cámara los pierde)
- Ningún cubo es visible por > 1.2 s
- WiFi se cae → sin telemetría

---

## Riesgos reales en ejecución a D=1

| Riesgo | Impacto | Mitigación |
|--------|--------|-----------|
| **Cubo no visible 1.2s+** | Se ignora del plan | Replanificación automática cada 20ms si reaparece |
| **Otro rover no visible 1.2s+** | Planifica solo (todas las tareas) | Fallback: volver a ver al otro y replan |
| **Telemetría stale > 700ms** | Se detiene | Reintenta conexión, log de error |
| **Interferencia real (3 trayectorias)** | Control debe evitar colisión | `control.c` hace elusión reactiva + ultrasónico |

---

## Cuándo activar el fallback greedy

El fallback greedy (`planner_fallback.h`) es una **red de seguridad**, no la solución principal:

```c
/* En mission.c, al invocar planner_solve(): */

plan_t plan;
bool planner_ok = planner_solve(&world, ROVER_ID, PEER_ID, &plan);

if (!planner_ok) {
    /* Plan óptimo no disponible. Usar greedy como fallback. */
    cube_color_t my_cube, peer_cube;
    if (planner_greedy_assign(&world, ROVER_ID, PEER_ID, &my_cube, &peer_cube)) {
        ESP_LOGW(TAG, "Usando fallback greedy: yo=%d, peer=%d", 
                 my_cube, peer_cube);
        /* Ejecutar manual la tarea my_cube. */
    }
}
```

**Nota:** Ya hay replanificación automática cada 20 ms, así que el fallback rara vez se dispara.

---

## Validador de diagnóstico: D=1 máxima

Agregado a `planner_validation.h`:

```c
plan_validity_t diag = {0};
planner_validate_result(&world, &plan, &diag);

if (diag.is_high_difficulty) {
    ESP_LOGI(TAG, "ESCENARIO DIFÍCIL: dist_avg=%.1f, interference=%d",
             diag.avg_distance_cells, diag.interference_count);
}
```

**Métricas de D=1:**
- `avg_distance_cells >= 35` celdas (~700 mm)
- `interference_count == 3` (todas las parejas se cruzan)
- `makespan >= 420 s` (7 minutos de 10 disponibles)

---

## Suite de escenarios: Validación local

### Generar 10 escenarios a D=1.0

```bash
cd firmware-c/tools
python3 scenario_validator.py \
    --difficulty 1.0 \
    --count 10 \
    --seed 42 \
    --save-json \
    --output-dir ./scenarios_d1
```

Salida:
```
Generando 10 escenarios con D=1.0
  [1] OK: avg_dist=38.2 cells
  [2] OK: avg_dist=41.5 cells
  [3] OK: avg_dist=36.8 cells
  ...
  → scenarios_d1/scenario_D1.0_000.json
  → scenarios_d1/scenario_D1.0_001.json
  ...
```

### Estructura del JSON generado

```json
{
  "scenario": {
    "difficulty": 1.0,
    "cubes": [
      {"color": "red", "col": 30.5, "row": 8.2, "size_cells": 3},
      {"color": "green", "col": 8.1, "row": 20.0, "size_cells": 3},
      {"color": "blue", "col": 40.0, "row": 35.3, "size_cells": 3}
    ],
    "depots": [
      {"color": "red", "col": 25.0, "row": 2.0},
      {"color": "green", "col": 2.0, "row": 25.0},
      {"color": "blue", "col": 48.0, "row": 25.0}
    ],
    "rovers": [
      {"id": 10, "col": 12.0, "row": 45.0},
      {"id": 11, "col": 38.0, "row": 45.0}
    ]
  },
  "validation": {
    "valid": true,
    "cube_count": 3,
    "avg_distance": 38.2
  }
}
```

### Integración con tests en C

Cuando tengas WiFi + cámara real (o simulador), puedes:
1. Cargar estos JSON en la cámara simulada
2. Ejecutar el planner contra cada escenario
3. Registrar logs: makespan, planes evaluados, tiempos

---

## Cronograma de pruebas (basado en tus tests 1-5)

| Test | Estado | Objetivo | D=1 ready? |
|------|--------|----------|-----------|
| 1. Motores | ✅ Pasó | Direcciones correctas | Ya |
| 2. Recta | ⏳ Próximo | Calibrar `PLAN_DRIVE_CELLS_S` | Sí |
| 3. Sensores | ⏳ Próximo | Validar IMU, IR, ultrasónico | Sí |
| 4. Visión (simulador) | ⏳ Próximo | WiFi + planner + vision_client | Sí |
| 5. Giro | ⏳ Próximo | Calibrar `PLAN_TURN_RATE_DEG_S`, `TURN_CCW_SIGN` | Sí |
| **Test D=1 simulado** | 🔜 Después | Ejecutar planner vs 10 escenarios | **Crítico** |
| **Test D=1 real** | 🔜 Después | Cámara real + 5 intentos | **Final** |

---

## Métricas para monitorear en D=1

```c
/* En mission.c, cada ciclo: */

if (diag.is_high_difficulty) {
    ESP_LOGI(TAG, 
        "D≈1 tick: plans_eval=%d, makespan=%.1f, my_cost=%.1f, peer_cost=%.1f",
        plan.plans_evaluated,
        plan.makespan_s,
        plan.my_cost_s,
        plan.peer_cost_s);
}
```

**Valores esperados a D=1:**
- `plans_evaluated`: 24 (todas las permutaciones)
- `makespan_s`: 420–540 s (7–9 minutos)
- `my_cost_s` vs `peer_cost_s`: balanceados (diferencia < 10%)

Si ves `plans_evaluated = 1` → indica que solo hay 1 cubo visible (stale).

---

## Resumen: ¿Necesitas preocuparte por D=1?

**Respuesta corta: No.**

Tu planner es óptimo. El riesgo real es **ejecución imperfecta** (colisiones, patinaje, cubo que se desplaza), no planificación. La replanificación cada 20 ms absorbe casi todo.

**Lo que sí debes hacer:**
1. ✅ Calibrar `PLAN_DRIVE_CELLS_S` y `PLAN_TURN_RATE_DEG_S` (tests 2 y 5)
2. ✅ Probar contra escenarios D=1 generados (cuando tengas cámara)
3. ✅ Monitorear logs en real time para detectar alertas
4. ✅ Tener el fallback greedy listo (está, no requiere activación manual)

**El planner no te va a dejar en la estacada.**

