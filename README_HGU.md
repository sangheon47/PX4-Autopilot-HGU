# CUAV 7-Nano용 IIM42652 RT 루프

이 브랜치는 CUAV 7-Nano에서 `IIM42652` 드라이버 기반 직접 실시간 루프를
돌리기 위해 필요한 최소 PX4 변경만 포함합니다.

## 포함된 변경 사항

- CUAV 7-Nano에서 `iim42652`를 활성화하고 사용하지 않는 내장 IMU를 비활성화함
- 부팅 시 보드 센서 초기화 스크립트에서 `iim42652`를 시작함
- 드라이버가 PWM/DSHOT 출력을 직접 쓰도록 기본 `pwm_out` 시작을 막음
- `IIM42652` 내부에 200 Hz 실시간 루프를 추가함
- 제어기 코드를 `src/lib/rt_control/`로 분리함
- UDP 텔레메트리 큐와 상태 출력 기능을 추가함

## 빠른 시작

```bash
git clone git@github.com:sangheon47/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
git switch share
make cuav_7-nano_default
```

빌드 결과물:

```bash
build/cuav_7-nano_default/cuav_7-nano_default.px4
```

## 먼저 볼 파일

- `boards/cuav/7-nano/default.px4board`
- `boards/cuav/7-nano/init/rc.board_sensors`
- `ROMFS/px4fmu_common/init.d/rcS`
- `src/drivers/imu/invensense/iim42652/IIM42652.cpp`
- `src/drivers/imu/invensense/iim42652/IIM42652.hpp`
- `src/lib/rt_control/rt_control.c`
- `src/lib/rt_control/rt_control.h`

## 빌드

```bash
make cuav_7-nano_default
```

## 업로드

보드가 연결되어 있고 PX4 업로드가 가능한 환경이면:

```bash
make cuav_7-nano_default upload
```

또는 생성된 `.px4` 파일을 QGroundControl로 업로드하면 됩니다.

## 부팅 동작

부팅 시 보드 센서 초기화 스크립트에서 아래 명령이 실행됩니다.

```bash
iim42652 -s -R 22 start
```

`rcS`에서는 기본 `pwm_out start`가 주석 처리되어 있고, `DSHOT`은 계속 PX4가
시작합니다.

## 실시간 루프 경로

`IIM42652::ControlLoopIRQ()` 내부 실행 순서는 아래와 같습니다.

1. `ReadSampleDirect()`
2. `rt_controller()`
3. `WriteStep()`
4. `TelemetryStep()`

주기 콜백은 `StartControlLoopIRQ()`에서 `hrt_call_every()`를 이용해
200 Hz(`CONTROL_PERIOD_US = 5000`)로 시작합니다.

## 주로 수정할 곳

- 제어기 로직: `src/lib/rt_control/rt_control.c`
- 루프 주기: `IIM42652.hpp`의 `CONTROL_PERIOD_US`
- PWM 범위: `IIM42652.hpp`의 `PWM_MIN_US`, `PWM_MAX_US`
- DSHOT 최대값: `IIM42652.hpp`의 `DSHOT_THROTTLE_MAX`
- UDP 목적지: `IIM42652.cpp`의 `InitUdpTelemetry()`

## 출력 매핑

- Servo 출력: 채널 1-4
- BLDC DSHOT 출력: 채널 5-6

## 부팅 후 확인

NSH 셸에서 아래 명령으로 확인합니다.

```bash
iim42652 status
```

출력에서 보게 되는 주요 항목:

- `RT period_us=5000`
- cycle 카운터
- input/control/output/exec 시간
- 가속도/자이로 값
- PWM/DSHOT 출력 값

## 권장 브랜치 구조

짧은 브랜치 이름 3개만 쓰는 것을 권장합니다.

- `share`: 외부 공유 및 재현용 기준 브랜치
- `team`: 팀이 함께 쓰는 공용 원격 브랜치
- `local`: 개인 작업 브랜치

이 포크를 직접 clone 했다면 기본 설정은 이렇게 시작하면 됩니다.

```bash
git switch share
git switch -c local
```

현재 저장소처럼 `px4fork` remote를 별도로 두고 쓰는 경우에는 아래처럼
시작하면 됩니다.

```bash
git fetch px4fork
git switch -c local px4fork/team
```

기본 흐름:

1. `local`에서 개인 작업
2. 테스트가 끝난 변경을 공용 원격 `team`에 반영
3. 안정화된 `team`을 `share`로 승격

## 팀원 협업 흐름

팀원도 같은 포크와 같은 짧은 브랜치 이름을 쓰면 됩니다.

```bash
git clone git@github.com:sangheon47/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
git switch share
git switch -c local
```

팀원은 `local`에서 작업하고, 빌드가 되는 상태가 되면 공용 원격 `team`에
반영합니다.

```bash
git fetch origin
git switch local
git merge origin/team
git push origin HEAD:team
```

## 팀원 변경을 내 로컬에 가져오기

팀원이 `team`에 새 커밋을 올렸다면, 내 `local`은 아래처럼 갱신하면 됩니다.

```bash
git fetch px4fork
git switch local
git merge px4fork/team
```

만약 내 `local`이 별도 커밋 없이 `team`만 따라가고 있다면 이것도 가능합니다.

```bash
git switch local
git pull --ff-only
```

## 검증된 변경 반영

내 `local` 작업을 팀에 반영할 때:

```bash
git fetch px4fork
git switch local
git merge px4fork/team
git push px4fork HEAD:team
```

`team`이 충분히 안정화되어 외부 공유 기준으로 올리고 싶을 때:

```bash
git fetch px4fork
git switch share
git merge --ff-only px4fork/team
git push px4fork share
```

## 다른 프로젝트나 다른 팀에서 재사용할 때

다른 사람이나 다른 팀이 이 저장소를 출발점으로 쓰고 싶다면, 이 저장소를
다시 fork해서 자기들만의 짧은 브랜치 구조를 유지하는 것이 좋습니다.

다른 팀 권장 구조:

- `share`: 그 팀의 안정 기준 브랜치
- `team`: 그 팀의 통합 브랜치
- `local`: 각 개발자의 개인 작업 브랜치

즉 구조는 이렇게 됩니다.

- 이 저장소는 이 저장소의 `share`, `team`, `local`을 유지
- 다른 팀은 자기들 fork를 새로 생성
- 그 fork 안에서 자기들만의 `share`, `team`, `local`을 운영

다른 팀 예시 시작 절차:

```bash
git clone git@github.com:<their-account>/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
git switch share
git switch -c local
```

## 참고

- 텔레메트리 큐는 single-producer single-consumer 구조로 구현되어 있어 IRQ
  루프와 non-IRQ flush 경로가 경쟁하지 않도록 되어 있습니다.
- `share`는 깨끗하게 유지하고, 협업은 `team`, 실험은 `local`에서 하는 것을
  권장합니다.
