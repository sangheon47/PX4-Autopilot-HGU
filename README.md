# PX4-Autopilot-7-Nano

`CUAV 7-Nano`와 `PX4 SITL`만 남긴 경량 PX4 레포입니다.

루트를 열면 가장 먼저 봐야 하는 파일은 `main.cpp`입니다.
이 파일이 7-nano 실시간 제어 메인 사이클이고, raw IMU register decode부터 PWM 계산까지 직접 담고 있습니다.
`IIM42652.cpp`는 IMU IRQ에서 SPI 전송과 telemetry/PWM 출력을 처리하는 하드웨어 래퍼입니다.
루프 주기, 샘플타임, 런타임, 정적 로그 배열 크기는 루트 `main.cpp` 상단 `#define` 블록에서 같이 바꿉니다.
실시간 제어 문맥은 [7NANO_CONTEXT.md](./7NANO_CONTEXT.md), 텔레메트리 포맷과 예시는 [TELEMETRY.md](./TELEMETRY.md)에 정리되어 있습니다.

지원 대상:

- `cuav_7-nano_minimal`
- `px4_sitl_default`

남겨둔 목적:

- 7-Nano 펌웨어 빌드 및 업로드
- IMU 기반 저수준 실시간 제어 실험
- MAVLink/QGC 연결
- Bullet Interceptor shell 기반 GZ SITL 확인

루트에서 의도적으로 제거한 것:

- 다른 보드들
- `qurt`, `ros2` 플랫폼
- 문서/테스트/CI/ROS 보조 파일
- `flightgear`, `gazebo-classic`, `jmavsim`, `jsbsim` 시뮬레이터
- 대부분의 GZ 예제 모델과 world

## Build

```bash
env CCACHE_DISABLE=1 make cuav_7-nano_minimal
make px4_sitl_default
```

생성물:

```bash
build/cuav_7-nano_minimal/cuav_7-nano_minimal.px4
build/px4_sitl_default/bin/px4
```

업로드:

```bash
make cuav_7-nano_minimal upload
```

## Main Paths

- `main.cpp`
- `include/control_main.h`
- `include/control_telemetry.h`
- `boards/cuav/7-nano`
- `boards/px4/sitl`
- `src/drivers/imu/invensense/iim42652/IIM42652.hpp`
- `src/drivers/imu/invensense/iim42652/IIM42652.cpp`
- `src/drivers/imu/invensense/iim42652/IIM42652_driver_main.cpp`
- `src/drivers/imu/invensense/iim42652/IIM42652_fast_loop.cpp`
- `src/drivers/imu/invensense/iim42652/IIM42652_motor_pwm.cpp`
- `src/drivers/imu/invensense/iim42652/IIM42652_telemetry.cpp`
- `src/drivers/imu/invensense/iim42652/IIM42652_cli.cpp`
- `ROMFS/px4fmu_common/init.d/rcS`
- `boards/cuav/7-nano/init/rc.minimal`
- `boards/cuav/7-nano/minimal.px4board`
- `Tools/simulation/gz/models/bullet_interceptor`
- `TELEMETRY.md`

가장 먼저 볼 파일:

- `main.cpp`: raw IMU register decode -> SI 단위 변환 -> 모터 명령 -> 4개 ESC PWM 목표 생성, loop Hz/period/sample time 설정
- `include/control_main.h`: main.cpp와 드라이버가 공유하는 최소 입출력 타입
- `include/control_telemetry.h`: telemetry queue와 shared telemetry 타입
- `src/drivers/imu/invensense/iim42652/IIM42652_fast_loop.cpp`: IMU 읽기 -> main.cpp 호출
- `src/drivers/imu/invensense/iim42652/IIM42652_motor_pwm.cpp`: 4개 모터 PWM 출력
- `src/drivers/imu/invensense/iim42652/IIM42652_telemetry.cpp`: queue -> uORB telemetry publish bridge
- `TELEMETRY.md`: 현재 telemetry field와 CLI/CSV 예시
- `ROMFS/px4fmu_common/init.d-posix/airframes/22000_gz_bullet_interceptor`: Bullet Interceptor SITL 시작점

## Remaining Runtime Pieces

7-nano minimal 빌드에 실제로 남겨둔 핵심은 아래 정도입니다.

- `iim42652`
- `gps`
- `bmp581`, `icp201xx`
- `ist8310`, `iis2mdc`
- `mavlink`, `dataman`
- `netman`, `param`, `perf`, `reboot`, `uorb`, `listener`, `ver`, `work_queue`

GZ 쪽은 아래만 남겨뒀습니다.

- `Tools/simulation/gz/models/bullet_interceptor`
- `Tools/simulation/gz/models/airspeed`
- `Tools/simulation/gz/worlds/default.sdf`

## Note

이 레포는 범용 PX4 개발용이 아니라, `7-Nano + 저수준 제어 + Bullet Interceptor SITL`만을 위해 줄여둔 작업용 트리입니다.
