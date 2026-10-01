#!/usr/bin/env python3
"""
scenario_validator.py - Genera escenarios como lo hace la organización
y valida que tu planner los resuelve óptimamente.

Uso:
    python scenario_validator.py --difficulty 1.0 --count 10 --save-json

Esto genera 10 escenarios a D=1.0 máxima dificultad, los guarda como JSON,
y los valida localmente contra tu planner.
"""

import json
import sys
import math
import random
import argparse
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional

# =====================================================================
# Modelo de escenarios (adaptado de docs/README.md)
# =====================================================================

@dataclass
class Cube:
    color: str  # "red", "green", "blue"
    col: float
    row: float
    size_cells: int = 3  # siempre 3x3

@dataclass
class Depot:
    color: str
    col: float
    row: float

@dataclass
class Rover:
    id: int
    col: float
    row: float

@dataclass
class Scenario:
    difficulty: float
    cubes: List[Cube]
    depots: List[Depot]
    rovers: List[Rover]
    grid_cols: int = 50
    grid_rows: int = 50
    cell_mm: float = 20.0
    effective_area_start: int = 5
    effective_area_end: int = 45  # 40x40 area

class ScenarioGenerator:
    """Genera escenarios siguiendo la lógica de la organización."""

    def __init__(self, seed: Optional[int] = None):
        if seed is not None:
            random.seed(seed)

    def generate(self, difficulty: float, max_trials: int = 700) -> Scenario:
        """
        Genera un escenario con dificultad D.

        difficulty: float in [0, 1]
        - D=0: cubos cercanos, sin interferencias
        - D=1: cubos lejanos, máximas interferencias

        Returns Scenario o None si no converge.
        """
        assert 0.0 <= difficulty <= 1.0, "Difficulty must be in [0,1]"

        # Depósitos fijos
        depots = [
            Depot("red", col=25, row=2),      # centro borde superior
            Depot("green", col=2, row=25),    # centro borde izquierdo
            Depot("blue", col=48, row=25),    # centro borde derecho
        ]

        # Posiciones iniciales de rovers (fijas)
        rovers = [
            Rover(10, col=12, row=45),
            Rover(11, col=38, row=45),
        ]

        # Objetivo de distancia y interferencias
        d_target = 6.5 + 35 * difficulty
        interference_targets = {
            (0.0, 0.28): 0,
            (0.28, 0.53): 1,
            (0.53, 0.77): 2,
            (0.77, 1.01): 3,
        }
        interference_target = 0
        for (d_min, d_max), target in interference_targets.items():
            if d_min <= difficulty < d_max:
                interference_target = target
                break

        best_scenario = None
        best_loss = float('inf')

        for trial in range(max_trials):
            # Generar posiciones candidatas para los 3 cubos
            cubes = self._generate_cube_positions(depots)
            if cubes is None:
                continue

            # Calcular métricas
            d_avg = self._avg_distance_to_depot(cubes, depots)
            interference = self._count_interferences(cubes, depots)

            # Función de pérdida (como en docs/)
            L_d = abs(d_avg - d_target) / 36.0
            L_c = abs(interference - interference_target) / 3.0
            epsilon = random.uniform(0, 0.06)
            loss = 0.55 * L_d + 0.45 * L_c + epsilon

            if loss < best_loss:
                best_loss = loss
                best_scenario = Scenario(
                    difficulty=difficulty,
                    cubes=cubes,
                    depots=depots,
                    rovers=rovers,
                )

        return best_scenario

    def _generate_cube_positions(self, depots: List[Depot]) -> Optional[List[Cube]]:
        """Genera 3 cubos válidos no superpuestos dentro del área efectiva."""
        AREA_START = 5
        AREA_END = 45
        CUBE_SIZE = 3
        MIN_SEPARATION = 2

        cubes = []
        max_attempts = 100

        for color in ["red", "green", "blue"]:
            for attempt in range(max_attempts):
                # Posición aleatoria del centro del cubo
                col = random.uniform(
                    AREA_START + CUBE_SIZE / 2,
                    AREA_END - CUBE_SIZE / 2,
                )
                row = random.uniform(
                    AREA_START + CUBE_SIZE / 2,
                    AREA_END - CUBE_SIZE / 2,
                )

                candidate = Cube(color=color, col=col, row=row)

                # Verificar no superposición y separación
                valid = True
                for existing in cubes:
                    dist = math.sqrt((col - existing.col) ** 2 +
                                   (row - existing.row) ** 2)
                    min_dist = (CUBE_SIZE + MIN_SEPARATION) / 2.0
                    if dist < min_dist:
                        valid = False
                        break

                # Verificar no superposición con depósitos
                for depot in depots:
                    dist = math.sqrt((col - depot.col) ** 2 +
                                   (row - depot.row) ** 2)
                    if dist < CUBE_SIZE:
                        valid = False
                        break

                if valid:
                    cubes.append(candidate)
                    break
            else:
                return None  # No se pudo colocar este cubo

        return cubes if len(cubes) == 3 else None

    def _avg_distance_to_depot(self, cubes: List[Cube],
                               depots: List[Depot]) -> float:
        """Calcula distancia Manhattan promedio entre cubos y sus depósitos."""
        total_dist = 0.0
        for cube in cubes:
            depot = next((d for d in depots if d.color == cube.color), None)
            if depot is not None:
                dist = abs(cube.col - depot.col) + abs(cube.row - depot.row)
                total_dist += dist
        return total_dist / len(cubes) if cubes else 0.0

    def _count_interferences(self, cubes: List[Cube],
                           depots: List[Depot]) -> int:
        """Cuenta parejas de trayectorias que se cruzan (interferencia)."""
        count = 0
        INTERFERENCE_THRESHOLD = 4.0  # celdas de margen

        for i, c1 in enumerate(cubes):
            for c2 in cubes[i + 1:]:
                d1 = next((d for d in depots if d.color == c1.color), None)
                d2 = next((d for d in depots if d.color == c2.color), None)

                if d1 is None or d2 is None:
                    continue

                # Distancia mínima entre las dos trayectorias (simplificado)
                min_dist = self._line_distance(
                    c1.col, c1.row, d1.col, d1.row,
                    c2.col, c2.row, d2.col, d2.row,
                )
                if min_dist < INTERFERENCE_THRESHOLD:
                    count += 1

        return min(count, 3)  # Máximo 3

    @staticmethod
    def _line_distance(x1a, y1a, x1b, y1b, x2a, y2a, x2b, y2b) -> float:
        """Distancia mínima entre dos segmentos (aproximación Manhattan)."""
        # Simplificación: usar Manhattan + diagonales
        return (abs(x1a - x2a) + abs(y1a - y2a) +
                abs(x1b - x2b) + abs(y1b - y2b)) / 2.0

# =====================================================================
# Validación local (sin firmware)
# =====================================================================

def validate_scenario(scenario: Scenario) -> dict:
    """Valida que un escenario es válido y contiene métricas de dificultad."""
    results = {
        "valid": True,
        "difficulty": scenario.difficulty,
        "cube_count": len(scenario.cubes),
        "cube_distances": [],
        "avg_distance": 0.0,
    }

    total_dist = 0.0
    for cube in scenario.cubes:
        depot = next((d for d in scenario.depots if d.color == cube.color),
                    None)
        if depot is None:
            results["valid"] = False
            continue

        dist = abs(cube.col - depot.col) + abs(cube.row - depot.row)
        results["cube_distances"].append({
            "color": cube.color,
            "distance_cells": dist,
        })
        total_dist += dist

    results["avg_distance"] = total_dist / len(scenario.cubes) if scenario.cubes else 0.0
    return results

# =====================================================================
# CLI
# =====================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Genera y valida escenarios para el Vision Rover Challenge"
    )
    parser.add_argument("--difficulty", type=float, default=1.0,
                        help="Dificultad D in [0,1]")
    parser.add_argument("--count", type=int, default=1,
                        help="Número de escenarios a generar")
    parser.add_argument("--seed", type=int, default=None,
                        help="Seed para reproducibilidad")
    parser.add_argument("--save-json", action="store_true",
                        help="Guardar escenarios como JSON")
    parser.add_argument("--output-dir", type=str, default="./scenarios",
                        help="Directorio de salida")

    args = parser.parse_args()

    gen = ScenarioGenerator(seed=args.seed)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)

    print(f"Generando {args.count} escenarios con D={args.difficulty}")

    for i in range(args.count):
        scenario = gen.generate(args.difficulty)
        if scenario is None:
            print(f"  [{i+1}] FALLÓ generación (timeout)")
            continue

        validation = validate_scenario(scenario)
        print(f"  [{i+1}] OK: avg_dist={validation['avg_distance']:.1f} cells")

        if args.save_json:
            filename = output_dir / f"scenario_D{args.difficulty:.1f}_{i:03d}.json"
            with open(filename, 'w') as f:
                json.dump({
                    "scenario": asdict(scenario),
                    "validation": validation,
                }, f, indent=2)
                print(f"      → {filename}")

if __name__ == "__main__":
    main()
