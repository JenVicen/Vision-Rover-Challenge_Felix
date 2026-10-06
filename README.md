# Equipo Felix — Vision Rover Challenge

Implementación del equipo **Felix** para el Vision Rover Challenge.

## Integrantes

- Jennifer Vicentes Valle
- Alejandro Espinoza Morales

## Solución

El sistema utiliza una cámara cenital para publicar el estado de la cancha y dos CenfoBots que toman decisiones de forma autónoma. Cada rover ejecuta en su ESP32 la planificación de tareas, navegación, coordinación entre pares y verificación de entregas.

Componentes principales:

- sistema de visión y contrato de telemetría en `vision-system/`;
- firmware ESP-IDF en `firmware-c/`;
- comunicación TCP para recibir el estado global;
- comunicación ESP-NOW entre los dos rovers;
- asignación exacta de cubos para minimizar el tiempo del último rover;
- control de rumbo con orientación de cámara y amortiguamiento giroscópico;
- verificación oficial de entrega mediante `in_depot` del protocolo v3.

## Documentación

| Documento | Contenido |
|---|---|
| [Guía de competencia](firmware-c/GUIA_COMPETENCIA.md) | Compilación, carga, puertos COM y lista de verificación para la ronda. |
| [Especificación técnica](firmware-c/ESPECIFICACION_TECNICA.md) | Arquitectura, planificación, control, coordinación y seguridad. |
| [Firmware](firmware-c/README.md) | Estructura del firmware y comandos básicos. |
| [Instalación en Windows](firmware-c/SETUP_NUEVA_PC.md) | Preparación de una computadora de desarrollo. |
| [Sistema de visión](vision-system/README.md) | Operación y configuración del publicador de telemetría. |
| [Contrato de telemetría](vision-system/contrato/CONTRATO.md) | Esquema NDJSON consumido por los rovers. |

## Estado de la entrega

Los entornos normales `rover10` y `rover11` compilan con PlatformIO y ESP-IDF. Antes de competir se debe ejecutar la validación física completa descrita en la guía de competencia y cargar nuevamente los dos entornos normales después de cualquier prueba de diagnóstico.
