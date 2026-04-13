---
name: px4-7nano-workflow
description: Use when working on PX4-Autopilot-7-nano tasks involving the 7-Nano realtime control loop, IIM42652 fast-path code, queue-based telemetry bridge, board bring-up, or Bullet Interceptor SITL. Use it for repo-specific file priorities, build commands, and editing constraints. Do not use it for generic Git, GitHub, or non-PX4 questions.
---

# PX4 7-Nano Workflow

## Start

1. Classify the task as `cuav_7-nano_minimal`, `px4_sitl_default`, or both.
2. Read the minimum entry points before proposing changes:
   - [../../../main.cpp](../../../main.cpp)
   - [../../../include/control_main.h](../../../include/control_main.h)
   - [../../../include/control_telemetry.h](../../../include/control_telemetry.h)
   - [../../../7NANO_CONTEXT.md](../../../7NANO_CONTEXT.md)
   - Relevant files under [../../../src/drivers/imu/invensense/iim42652](../../../src/drivers/imu/invensense/iim42652)
3. If the request is broad or under-specified, clarify whether the user wants firmware-path work, SITL-path work, or both before refactoring.
4. For repo intent and user preferences, read [../../../README.md](../../../README.md) and [../../../7NANO_CONTEXT.md](../../../7NANO_CONTEXT.md).

## Guardrails

- Keep the visible control logic in `main.cpp`.
- Do not invent controller equations, hidden helper layers, or placeholder architecture that the user did not ask for.
- Prefer direct PWM-per-motor edits first, then controller structure later when requested.
- Keep editable loop timing and sample-time values in `main.cpp`.
- Keep visible motor output commands in `main.cpp` as `motor_pwm_pct`, then convert to `motor_pwm_us`.
- Keep telemetry queue-based and out of the tight realtime path when possible.
- Keep `rt_control_telemetry` compact. Prefer only experiment-useful values: time base, IMU state, final motor PWM output, and external vision pose/age/validity.
- This repository is intentionally pruned. Do not reintroduce deleted boards, platforms, simulators, docs, or CI scaffolding unless explicitly requested.
- Preserve the SITL path, but treat 7-Nano low-level work as the main priority when tradeoffs appear.

## Validation

- Firmware-only changes: `env CCACHE_DISABLE=1 make cuav_7-nano_minimal`
- SITL-only changes: `make px4_sitl_default`
- Shared changes: run both when practical
- If you skip validation, say what blocked it

## Style Notes

- Keep code readable from `main.cpp` first.
- Prefer short, obvious names for local concepts.
- Use unit comments like `[Hz]`, `[us]`, `[s]`, `[%]`, `[m/s^2]` where they reduce ambiguity.
- Keep unit conversions on the same line when practical.
