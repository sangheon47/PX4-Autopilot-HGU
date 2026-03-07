# IIM42652 RT Loop for CUAV 7-Nano

This branch contains the minimum PX4 changes required to run a direct realtime
loop on the CUAV 7-Nano with the IIM42652 driver.

## Included changes

- Enable `iim42652` on CUAV 7-Nano and disable the unused onboard IMUs.
- Start `iim42652` from the board sensor init script at boot.
- Disable the default `pwm_out` startup so the driver can write PWM/DSHOT
  outputs directly.
- Add a 200 Hz realtime loop inside `IIM42652`.
- Split controller logic into `src/lib/rt_control/`.
- Add UDP telemetry queueing and status reporting.

## Files to inspect first

- `boards/cuav/7-nano/default.px4board`
- `boards/cuav/7-nano/init/rc.board_sensors`
- `ROMFS/px4fmu_common/init.d/rcS`
- `src/drivers/imu/invensense/iim42652/IIM42652.cpp`
- `src/drivers/imu/invensense/iim42652/IIM42652.hpp`
- `src/lib/rt_control/rt_control.c`
- `src/lib/rt_control/rt_control.h`

## Build

```bash
make cuav_7-nano_default
```

## Boot behavior

At boot the board sensor script starts:

```bash
iim42652 -s -R 22 start
```

The default `pwm_out start` in `rcS` is commented out, and DSHOT is still
started by PX4.

## Realtime path

Runtime flow inside `IIM42652::ControlLoopIRQ()`:

1. `ReadSampleDirect()`
2. `rt_controller()`
3. `WriteStep()`
4. `TelemetryStep()`

The periodic callback is started by `StartControlLoopIRQ()` using
`hrt_call_every()` at 200 Hz (`CONTROL_PERIOD_US = 5000`).

## What to edit

- Controller logic: `src/lib/rt_control/rt_control.c`
- Loop rate: `CONTROL_PERIOD_US` in `IIM42652.hpp`
- PWM range: `PWM_MIN_US`, `PWM_MAX_US` in `IIM42652.hpp`
- DSHOT max: `DSHOT_THROTTLE_MAX` in `IIM42652.hpp`
- UDP destination: `InitUdpTelemetry()` in `IIM42652.cpp`

## Output mapping

- Servo outputs: channels 1-4
- BLDC DSHOT outputs: channels 5-6

## Check after boot

From the NSH shell:

```bash
iim42652 status
```

Expected status output shows:

- `RT period_us=5000`
- cycle counter
- input/control/output/exec timing
- accel/gyro values
- PWM and DSHOT outputs

## Notes

- The current `rt_controller()` is a stub controller that outputs fixed
  normalized commands.
- The telemetry queue is implemented as a single-producer single-consumer
  queue so the IRQ loop and the non-IRQ flush path do not race.
