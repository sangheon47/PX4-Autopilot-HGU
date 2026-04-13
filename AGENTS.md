# PX4-Autopilot-7-Nano

## Repository Focus

- This repository is a trimmed PX4 tree for `cuav_7-nano_minimal` and `px4_sitl_default`.
- Prioritize the 7-Nano realtime control path. Keep the Bullet Interceptor GZ SITL path alive, but do not let SITL-only cleanup break board work.
- Do not restore removed boards, platforms, simulators, CI, ROS, or broad PX4 subsystems unless the user explicitly asks.

## Start Here

- Read [README.md](./README.md) for the trimmed-repo scope.
- Read [7NANO_CONTEXT.md](./7NANO_CONTEXT.md) for user preferences and current workflow intent.
- For most firmware tasks, inspect these files before editing:
  - [main.cpp](./main.cpp)
  - [include/control_main.h](./include/control_main.h)
  - [include/control_telemetry.h](./include/control_telemetry.h)
  - [7NANO_CONTEXT.md](./7NANO_CONTEXT.md)
  - Relevant files under [src/drivers/imu/invensense/iim42652](./src/drivers/imu/invensense/iim42652), especially:
    - `IIM42652_driver_main.cpp`
    - `IIM42652_fast_loop.cpp`
    - `IIM42652_motor_pwm.cpp`
    - `IIM42652_telemetry.cpp`
    - `IIM42652_cli.cpp`
    - `IIM42652.cpp`
    - `IIM42652.hpp`

## Task Routing

- First decide whether the task affects `cuav_7-nano_minimal`, `px4_sitl_default`, or both.
- If the request is ambiguous, clarify which path is the priority before making wide changes.
- When the task is specifically about the 7-Nano realtime loop, IMU fast path, queue-based telemetry bridge, or Bullet Interceptor SITL, use the repo skill `px4-7nano-workflow`.

## Editing Rules

- Keep the visible realtime control path in `main.cpp`.
- Do not add placeholder controller logic or speculative abstractions ahead of the user.
- Prefer short, clear local names over extra wrappers when the scope is obvious.
- Keep editable timing, loop rate, sample-time, and related constants in `main.cpp`.
- Keep visible motor output commands in `main.cpp` as `motor_pwm_pct` and convert to `motor_pwm_us`.
- Keep telemetry queue-based and outside the tight realtime path when possible.
- Keep `rt_control_telemetry` compact. Prefer only experiment-useful values: time base, IMU state, final motor PWM output, and external vision pose/age/validity.
- Preserve readability from `main.cpp` first.
- Use unit comments like `[Hz]`, `[us]`, `[s]`, `[%]`, `[m/s^2]` when they improve scanability.
- When a line performs a unit conversion, keep the conversion on that same line when practical.

## Validation

- Firmware path changes: run `env CCACHE_DISABLE=1 make cuav_7-nano_minimal`.
- SITL path changes: run `make px4_sitl_default`.
- Shared changes: run both builds when practical.
- If validation is skipped, explain exactly why.
- Do not delete `build/cuav_7-nano_minimal/` just to silence editor or tooling issues.
