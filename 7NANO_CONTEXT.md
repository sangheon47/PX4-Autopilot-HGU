# 7-Nano Context

## Repo
- This repo is `PX4-Autopilot-7-nano`.
- Work here for the 7-nano custom stack, not in the team PX4 repo.
- Current branch intent: lightweight 7-nano only.

## Goal
- Keep only what is needed for:
- IMU/GPS/baro/mag/TELEM communication
- low-level realtime loop
- direct PWM output to 4 ESC / 4 motors
- QGC connection
- simulation path kept alive

## Open These First
- [main.cpp](./main.cpp)
- [include/control_main.h](./include/control_main.h)
- [include/control_telemetry.h](./include/control_telemetry.h)

## Current Realtime Flow
- `main.cpp` is the file the user wants to see first.
- Current visible flow in `main.cpp`:
- raw IMU bytes -> raw sensor values -> unit conversion -> direct motor PWM commands -> PWM clamp/output values
- The user does not want placeholder controller logic added ahead of time.
- For now, each motor should be commandable directly with PWM in `main.cpp`.

## Folder Intent
- `main.cpp`: visible realtime control loop
- `include/`: shared custom types and telemetry queue only
- `src/drivers/imu/invensense/iim42652/`: low-level IMU read, fast-loop scheduling, actuator write, bridge, CLI

## User Preferences
- Keep code visually simple and obvious.
- Avoid long PX4-style names when a shorter clear name works.
- Do not add unnecessary wrappers/helpers.
- Put editable timing settings in `main.cpp`.
- Use unit comments like `[Hz]`, `[us]`, `[s]`, `[m/s^2]`.
- When a line converts units, write the unit conversion on that same line.
- Do not jump ahead and invent controller equations before they exist.
- Prefer direct PWM per motor first, then controller later.

## Build
- Build command:
```bash
env CCACHE_DISABLE=1 make cuav_7-nano_minimal
```
- Keep `build/cuav_7-nano_minimal/` when using VSCode.
- `.vscode/c_cpp_properties.json` points IntelliSense to `build/cuav_7-nano_minimal/compile_commands.json`.
- If VSCode still shows stale include errors:
```text
Reload Window
C/C++: Reset IntelliSense Database
```

## Simulation
- The shell model the user referred to is `BULLET_INTERCEPTOR`.
- Simulation support should stay alive, but the low-level 7-nano path remains the main priority.

## Notes For Future Work
- GPS, baro, mag, RC can be added later, but the visible control path should stay simple.
- Telemetry is queue-based and should remain outside the tight realtime path as much as possible.
- If changing comments or names, keep the code readable from `main.cpp` first.
