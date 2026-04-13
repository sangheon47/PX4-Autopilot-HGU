# 7-Nano Context

Last updated: 2026-04-14

## Repo
- This repo is `PX4-Autopilot-7-nano`.
- Work here for the 7-nano custom stack, not in the team PX4 repo.
- Current branch intent: lightweight 7-nano only.

## Goal
- Keep only what is needed for IMU/GPS/baro/mag/TELEM communication.
- Keep the low-level realtime loop alive on `cuav_7-nano_minimal`.
- Keep direct PWM output to 4 ESC / 4 motors simple and visible.
- Keep QGC connection and the simulation path alive.

## Open These First
- [main.cpp](./main.cpp)
- [include/control_main.h](./include/control_main.h)
- [include/control_telemetry.h](./include/control_telemetry.h)
- [src/drivers/imu/invensense/iim42652/IIM42652.hpp](./src/drivers/imu/invensense/iim42652/IIM42652.hpp)
- [src/drivers/imu/invensense/iim42652/IIM42652_fast_loop.cpp](./src/drivers/imu/invensense/iim42652/IIM42652_fast_loop.cpp)
- [src/drivers/imu/invensense/iim42652/IIM42652_motor_pwm.cpp](./src/drivers/imu/invensense/iim42652/IIM42652_motor_pwm.cpp)
- [src/drivers/imu/invensense/iim42652/IIM42652_telemetry.cpp](./src/drivers/imu/invensense/iim42652/IIM42652_telemetry.cpp)

## Current Realtime Flow
- `main.cpp` is the file the user wants to read and edit first.
- Visible flow in `main.cpp` is:
- raw IMU bytes -> raw sensor values -> unit conversion -> motor PWM percent commands -> motor PWM microseconds
- The user does not want placeholder controller logic added ahead of time.
- For now, each motor should be commandable directly in `main.cpp`.

## Current Timing Convention
- Keep editable timing settings in [main.cpp](./main.cpp).
- Current timing values are:
- `SAMPLE_FREQ` `[Hz]`
- `SAMPLE_PERIOD` `[us]` derived from `SAMPLE_FREQ`
- `SAMPLE_DT` `[s]` derived from `SAMPLE_PERIOD`
- `SAMPLE_PERIOD` is an integer scheduler period in microseconds, so non-divisor frequencies of `1,000,000` will truncate.

## Current Motor PWM Convention
- Use `motor_pwm_*` names, not `esc_*`, in the custom control path.
- Use `motor_pwm_pct` for the visible per-motor command in [main.cpp](./main.cpp).
- Use `motor_pwm_us` for the actual PWM pulse width sent to the output layer.
- Keep unit comments on editable and converted values:
- `[%]` for percent
- `[us]` for PWM pulse width
- The current shared custom structs are:
- `control_input_t.motor_pwm_min_us`
- `control_input_t.motor_pwm_max_us`
- `control_output_t.motor_pwm_pct`
- `control_output_t.motor_pwm_us`
- The current telemetry topic fields are:
- `rt_control_telemetry.motor_pwm_us`

## Current Compact Telemetry
- The realtime telemetry was intentionally reduced to experiment-useful values only.
- Redundant fields such as `cycle`, `actual_start_s`, `input_us`, `control_us`, `output_us`, `missed_cycles`, `motor_pwm_pct`, and `opti_seq` were removed from `rt_control_telemetry`.
- Current `rt_control_telemetry` fields:
- `timestamp` `[us]`: control-cycle start time since boot
- `loop_dt_us` `[us]`: actual interval since the previous control cycle
- `exec_us` `[us]`: total realtime-loop execution time
- `accel_m_s2[3]` `[m/s^2]`: latest control-loop acceleration
- `gyro_rad_s[3]` `[rad/s]`: latest control-loop angular rate
- `motor_pwm_us[4]` `[us]`: final PWM sent to the 4 motor ESC outputs
- `vision_pos_m[3]` `[m]`: latest visual odometry position, `NaN` if invalid
- `vision_rpy_rad[3]` `[rad]`: latest visual odometry roll/pitch/yaw, `NaN` if invalid
- `vision_age_us` `[us]`: age of the latest visual odometry sample
- `vision_valid` `[0/1]`: whether the visual odometry sample is finite and fresh
- The MAVLink tunnel payload was reduced accordingly and now includes `timestamp_us` as well.

## Current IMU Driver File Map
- `IIM42652_driver_main.cpp`: module entry / CLI dispatch glue
- `IIM42652.cpp`: core driver lifecycle and state machine
- `IIM42652_fast_loop.cpp`: fast realtime loop scheduling and execution
- `IIM42652_motor_pwm.cpp`: direct motor PWM output path
- `IIM42652_telemetry.cpp`: slow-side publish bridge and external input cache
- `IIM42652_cli.cpp`: status output and CLI custom commands
- `IIM42652.hpp`: private driver state and function declarations

## Folder Intent
- `main.cpp`: visible realtime control logic
- `include/`: shared custom types and telemetry queue only
- `src/drivers/imu/invensense/iim42652/`: low-level IMU read, fast-loop scheduling, motor PWM write, telemetry bridge, CLI
- Do not move `.cpp` implementation files into `include/`. Keep declarations in headers and implementations in `src/`.

## User Preferences
- Keep code visually simple and obvious.
- Avoid long PX4-style names when a shorter clear name works.
- Do not add unnecessary wrappers or speculative helper layers.
- Use unit comments like `[Hz]`, `[us]`, `[s]`, `[%]`, `[m/s^2]`.
- When a line converts units, keep the unit conversion on that same line when practical.
- Do not jump ahead and invent controller equations before they exist.
- Prefer direct per-motor PWM work first, then controller structure later.
- If renaming, prefer names that read cleanly from `main.cpp` first.

## Build And Tooling
- Firmware build command:
```bash
env CCACHE_DISABLE=1 make cuav_7-nano_minimal
```
- Keep `build/cuav_7-nano_minimal/` when using VS Code.
- `.vscode/c_cpp_properties.json` points IntelliSense to `build/cuav_7-nano_minimal/compile_commands.json`.
- This repo may be opened with either `ms-vscode.cpptools` or `clangd`.
- If VS Code still shows stale include or member errors after valid code changes:
```text
C/C++: Reset IntelliSense Database
C/C++: Rescan Workspace
Developer: Reload Window
```
- If those `C/C++:` commands do not appear, check that `ms-vscode.cpptools` is enabled and that a C/C++ file is focused.
- If the editor is using `clangd` instead, use:
```text
Clangd: Restart language server
Developer: Reload Window
```
- Do not delete `build/cuav_7-nano_minimal/` just to silence editor errors.

## Validation Status
- The current `motor_pwm_*` rename and the `IIM42652_*` file renames were built successfully on 2026-04-14 with:
```bash
env CCACHE_DISABLE=1 make cuav_7-nano_minimal
```

## Simulation
- The shell model the user referred to is `BULLET_INTERCEPTOR`.
- Simulation support should stay alive, but the low-level 7-nano path remains the main priority.

## Notes For Future Work
- GPS, baro, mag, RC can be added later, but the visible control path should stay simple.
- Telemetry is queue-based and should remain outside the tight realtime path as much as possible.
- A later control split can add:
- manual mode with Wi-Fi keyboard thrust input
- auto mode with real sensor-based autonomous movement / waypoint flight
- A future control-mode split should likely keep `main.cpp` visible while selecting between:
- `manual`
- `timed_test`
- `auto`
- Timed maneuvers should use an internal elapsed-time gate or cycle counter in control state.
- Do not add a telemetry cycle counter unless an experiment explicitly needs to log it.
- A later trigger path can come from CLI, MAVLink, Wi-Fi, or another external command source, but the control code should stay readable from `main.cpp`.
- If changing comments, names, or structure, keep readability from `main.cpp` first.
