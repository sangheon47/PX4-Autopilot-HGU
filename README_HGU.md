# CUAV 7-Nano용 IIM42652 RT 루프

이 브랜치는 CUAV 7-Nano에서 `IIM42652` 드라이버 기반 직접 실시간 루프를
돌리기 위해 필요한 최소 PX4 변경만 포함합니다.

## 포함된 변경 사항

- CUAV 7-Nano에서 `iim42652`를 활성화하고 사용하지 않는 내장 IMU를 비활성화함
- 부팅 시 보드 센서 초기화 스크립트에서 `iim42652`를 시작함
- 드라이버가 PWM/DSHOT 출력을 직접 쓰도록 기본 `pwm_out` 시작을 막음
- 표준 `dshot start`를 비활성화하고 `iim42652`가 DShot 출력 5-6을 직접 소유하도록 정리함
- `IIM42652` 내부에 200 Hz 실시간 루프를 추가함
- 제어기 코드를 `src/lib/rt_control/`로 분리함
- `rt_control_telemetry` uORB + MAVLink TUNNEL 스트림을 추가함
- `iim42652 motor auto|stop|status|set <0..1>` 콘솔 명령을 추가함
- DShot startup hold와 기본 `stop` 모드를 추가함

## 빠른 시작

```bash
git clone git@github.com:sangheon47/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
# 현재 브랜치가 team이 아니라면 아래 줄 실행
# git switch --track origin/team
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

## VS Code 빨간 줄 정리

`IIM42652.cpp`, `IIM42652.hpp` 같은 PX4 보드 전용 파일은 에디터가 실제
빌드 설정을 못 읽으면 빨간 줄이 남을 수 있습니다. 이 저장소는 기본적으로
`cuav_7-nano_default` 기준으로 맞춰져 있습니다.

처음 한 번은 아래 순서로 맞추는 것이 좋습니다.

```bash
make cuav_7-nano_default
ln -sfn build/cuav_7-nano_default/compile_commands.json compile_commands.json
```

그 다음 VS Code에서 아래를 실행합니다.

1. `CMake: Select Variant` -> `cuav_7-nano_default`
2. `CMake: Delete Cache and Reconfigure`
3. `C/C++: Reset IntelliSense Database`
4. `Developer: Reload Window`

실제 오류 확인은 아래 둘 중 하나로 합니다.

```bash
make cuav_7-nano_default
make cuav_7-nano_default upload
```

또는 VS Code에서 `Run Task`로 아래 task를 실행하면 됩니다.

- `cuav_7-nano build check`
- `cuav_7-nano upload`

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

이후 `IIM42652::InitActuatorDirect()`가 아래를 직접 수행합니다.

- Servo 출력 1-4 초기화
- DShot 출력 5-6 초기화 및 arm
- `motor_stop` 전송
- 약 3초 startup hold 시작

현재 `rcS`에서는 표준 `dshot start`를 비활성화해 두었습니다. `iim42652`와 표준
`dshot`가 동시에 같은 출력 자원을 잡으면 콘솔 명령이 먹지 않는 상태가 생길 수
있기 때문입니다.

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
- 텔레메트리 MAVLink 스트림: `src/modules/mavlink/streams/RT_CONTROL_TELEMETRY.hpp`

## 출력 매핑

- Servo 출력: 채널 1-4
- BLDC DSHOT 출력: 채널 5-6

## TELEM1 MAVLink 텔레메트리 수신

현재 `iim42652` 텔레메트리는 `rt_control_telemetry` uORB 토픽으로 publish 되고,
MAVLink `TUNNEL` 메시지로 `TELEM1` 링크를 통해 송신됩니다.

사전 설정:

```bash
param set MAV_0_CONFIG 101
param set MAV_0_MODE 2
param set MAV_0_RATE 80000
param set SER_TEL1_BAUD 921600
param save
reboot
```

의미:

- `MAV_0_CONFIG 101`: `TELEM1`를 MAVLink instance 0으로 사용
- `MAV_0_MODE 2`: companion/onboard용 기본 스트림 세트 사용
- `MAV_0_RATE 80000`: 200 Hz RT 텔레메트리와 기본 MAVLink 스트림을 같이 보내기 위한 송신 rate
- `SER_TEL1_BAUD 921600`: TELEM1 UART 보레이트

수신 PC에서는 아래 스크립트로 저장합니다.

UDP 브리지 모드 예시:

```bash
python3 Tools/telem_csv_logger.py --mode udp --udp-port 14550 --output logs/rt_telem.csv
```

직렬 직접 수신 예시:

```bash
python3 Tools/telem_csv_logger.py --mode serial --serial-device /dev/ttyUSB0 --serial-baud 921600 --output logs/rt_telem.csv
```

생성된 CSV는 MATLAB `readtable()` 또는 Python `pandas.read_csv()`로 바로 읽을 수 있습니다.

DroneBridge 설정은 아래처럼 두는 것이 맞습니다.

- `UART serial protocol`: `MAVLink`
- `UART baud`: `921600`
- UDP 수신 포트: 보통 `14550`

## 수동 BLDC 제어

MAVLink Console 또는 NSH 셸에서 아래 명령을 사용합니다.

```bash
iim42652 motor status
iim42652 motor stop
iim42652 motor set 0.03
iim42652 motor set 0.05
iim42652 motor set 0.10
iim42652 motor auto
```

의미:

- `status`: 현재 BLDC 모드와 수동 설정값 확인
- `stop`: BLDC 즉시 정지
- `set <0..1>`: BLDC를 지정 출력으로 고정
- `auto`: `rt_controller()` 출력으로 복귀

권장 시험 순서:

```bash
iim42652 motor status
iim42652 motor set 0.03
# 필요하면 0.05, 0.10 등으로 증가
iim42652 motor stop
```

## BLDC 모드 의미

- `stop`: BLDC에 `motor_stop`을 보냄
- `set`: 지정한 수동 출력값을 사용
- `auto`: `rt_controller()`가 만든 BLDC 값을 그대로 사용

현재 `src/lib/rt_control/rt_control.c`에서는 BLDC 출력이 `0.5`, `0.5`로
고정되어 있으므로, 지금의 `auto`는 사실상 50% 고정 출력처럼 동작합니다.

## 부팅 후 확인

NSH 셸에서 아래 명령으로 확인합니다.

```bash
iim42652 status
```

출력에서 보게 되는 주요 항목:

- `RT period_us=5000`
- `manual_mode`
- `RT telem topic=rt_control_telemetry advertised=true`
- `manual`
- `hold_active`
- cycle 카운터
- input/control/output/exec 시간
- 가속도/자이로 값
- PWM/DSHOT 출력 값

상태 확인 팁:

- 수동 제어 상태 확인은 `iim42652 motor status`가 가장 직접적입니다.
- `iim42652 status`의 `manual_mode`, `manual`, `bldc_dshot`도 참고할 수 있습니다.
- 현재 `iim42652 status`의 BLDC 퍼센트 표시는 최종 override 출력과 완전히
  일치하지 않을 수 있으므로, 수동 출력 확인은 `motor status`와 실제
  `bldc_dshot` 값을 함께 보는 편이 낫습니다.

## 권장 브랜치 구조

기본 운영은 짧은 브랜치 이름 2개만 쓰는 것을 권장합니다.

- `team`: 팀이 함께 쓰는 메인 작업 브랜치
- `share`: 필요할 때만 남겨두는 보존용 기준 브랜치

즉 평소에는 거의 전부 `team`에서 작업하고, `share`는 꼭 필요할 때만 유지하는
방식이 가장 단순합니다.

## 팀 작업 시작

이 포크를 직접 clone 했다면 보통 이렇게 시작하면 됩니다.

```bash
git clone git@github.com:sangheon47/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
# 현재 브랜치가 team이 아니라면 아래 줄 실행
# git switch --track origin/team
git pull --ff-only
```

기본 흐름:

1. `team`에서 작업
2. 커밋 후 `team`에 push
3. 정말 기준점을 남겨야 할 때만 `share` 갱신

## 팀원 협업 흐름

팀원도 같은 저장소를 clone 한 뒤 `team`에서 바로 작업하면 됩니다.

```bash
git clone git@github.com:sangheon47/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
# 현재 브랜치가 team이 아니라면 아래 줄 실행
# git switch --track origin/team
git pull --ff-only
```

작업 후 반영:

```bash
git add <files>
git commit -m "원하는 커밋 메세지"
git push origin team
```

## 팀원 변경 받아오기

다른 팀원이 `team`에 새 커밋을 올렸다면, 아래처럼 받아오면 됩니다.

```bash
git switch team
git pull --ff-only
```

## 임시 브랜치가 필요할 때

개인 실험을 `team`에 바로 올리고 싶지 않다면, 그때만 임시 브랜치를 만들면
됩니다. 이름은 `temp1`, `temp2`처럼 새 번호를 붙여서 사용하면 됩니다.

```bash
git switch team
git pull --ff-only
git switch -c temp1
```

정리 후 다시 `team`에 반영:

```bash
git switch team
git merge temp1
git push origin team
```

## 검증된 변경 반영

`team`이 충분히 안정화되어 별도 기준점을 남기고 싶을 때만:

```bash
git fetch origin
git switch share
git merge --ff-only origin/team
git push origin share
git switch team
```

## 다른 프로젝트나 다른 팀에서 재사용할 때

다른 사람이나 다른 팀이 이 저장소를 출발점으로 쓰고 싶다면, 이 저장소를
다시 fork해서 자기들만의 짧은 브랜치 구조를 유지하는 것이 좋습니다.

다른 팀 권장 구조:

- `team`: 그 팀의 메인 작업 브랜치
- `share`: 그 팀이 필요할 때만 남겨두는 기준 브랜치
- 필요하면 임시 브랜치만 추가 사용

즉 구조는 이렇게 됩니다.

- 이 저장소는 이 저장소의 `share`, `team`을 유지
- 다른 팀은 자기들 fork를 새로 생성
- 그 fork 안에서 자기들만의 `share`, `team`을 운영

다른 팀 예시 시작 절차:

```bash
git clone git@github.com:<their-account>/PX4-Autopilot-HGU.git
cd PX4-Autopilot-HGU
# 현재 브랜치가 team이 아니라면 아래 줄 실행
# git switch --track origin/team
```

## 참고

- 텔레메트리 큐는 single-producer single-consumer 구조로 구현되어 있어 IRQ
  루프와 non-IRQ flush 경로가 경쟁하지 않도록 되어 있습니다.
- 평소 작업은 `team`에서 직접 진행하고, `share`는 꼭 필요할 때만 갱신하는
  것을 권장합니다.
