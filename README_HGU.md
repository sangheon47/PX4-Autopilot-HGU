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

## Quick start

```bash
git clone git@github.com:sangheon47/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
git switch share
make cuav_7-nano_default
```

Build output:

```bash
build/cuav_7-nano_default/cuav_7-nano_default.px4
```

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

## Flash

If your board is connected and PX4 upload is available:

```bash
make cuav_7-nano_default upload
```

Or flash the generated `.px4` file with QGroundControl.

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

## Recommended branch model

Use three short branch names:

- `share`: clean reference branch for sharing and reproduction
- `team`: integration branch used by you and your teammate
- `local`: your personal working branch

If you cloned this fork directly, a simple setup is:

```bash
git switch share
git switch -c local
```

If you use this repository as a local workspace with the fork configured as
`px4fork`, a simple setup is:

```bash
git fetch px4fork
git switch -c share px4fork/share
git switch -c team px4fork/team
git switch -c local px4fork/team
```

Typical flow:

1. Do your own edits on `local`
2. Merge tested changes into `team`
3. Promote stable `team` changes into `share`

## Teammate workflow

Your teammate can use the same fork and the same short branch names:

```bash
git clone git@github.com:sangheon47/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
git switch -c team origin/team
git switch -c local
```

Work on `local`, then merge into `team` when the build is good:

```bash
git switch team
git merge local
git push origin team
```

## Updating your local branch from teammate changes

If your teammate pushed new commits to `team`, update your own `local` branch
like this:

```bash
git fetch px4fork
git switch local
git merge px4fork/team
```

If your `local` branch has no extra commits and only follows `team`, this also
works:

```bash
git switch local
git pull --ff-only
```

## Promoting tested changes

When your `local` work is ready to share with the team:

```bash
git switch team
git merge local
git push px4fork team
git switch local
```

When `team` is stable and you want a clean shared reference:

```bash
git switch share
git merge --ff-only team
git push px4fork share
git switch local
```

## Reusing this work in another project or team

If another person or another team wants to use this repository as a starting
point, they should fork this fork and keep their own short branch structure.

Recommended model for another team:

- `share`: their stable reference branch
- `team`: their team integration branch
- `local`: each developer's personal working branch

That means:

- your repository keeps your own `share`, `team`, and `local`
- another team creates their own fork
- that fork gets its own `share`, `team`, and `local`

Do not ask unrelated teams to push directly into your `team` branch. Treat
your `share` branch as the published baseline and let other teams branch from
their own fork.

Example flow for another team:

```bash
git clone git@github.com:<their-account>/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
git switch share
git switch -c local
```

## Notes

- The current `rt_controller()` is a stub controller that outputs fixed
  normalized commands.
- The telemetry queue is implemented as a single-producer single-consumer
  queue so the IRQ loop and the non-IRQ flush path do not race.
- Keep `share` clean, use `team` for collaboration, and do experiments on
  `local`.
