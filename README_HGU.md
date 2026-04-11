# CUAV 7-Nano용 IIM42652 RT 루프

이 브랜치는 CUAV 7-Nano에서 `IIM42652` 드라이버 기반 직접 실시간 루프를
돌리기 위해 필요한 최소 PX4 변경만 포함합니다.

## 포함된 변경 사항

- CUAV 7-Nano에서 `iim42652`를 활성화하고 사용하지 않는 내장 IMU를 비활성화함
- 부팅 시 보드 센서 초기화 스크립트에서 `iim42652`를 시작함
- 드라이버가 PWM 출력을 직접 쓰도록 기본 `pwm_out` 시작을 막음
- `iim42652`가 MR-X4 ESC용 PWM 출력 1-4를 직접 소유하도록 정리함
- `IIM42652` 내부에 200 Hz 실시간 루프를 추가함
- 제어기 코드를 `src/lib/rt_control/`로 분리함
- `rt_control_telemetry` uORB + MAVLink TUNNEL 스트림을 추가함
- MR-X4 안내에 맞춰 PWM 출력 주파수를 `250 Hz`로 고정함
- 실험용 모터 구동 명령을 제거하고 `iim42652 esc_calib high|low|status`와 제한된 `iim42652 esc_test <0..10>`만 남김

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

- PWM 출력 1-4 초기화
- MR-X4 ESC 입력 주파수를 250 Hz로 설정
- 저스로틀(`1000 us`) 출력으로 ESC 초기화

현재 `rcS`에서는 표준 `pwm_out start`와 `dshot start`를 모두 비활성화해 두었습니다.
`iim42652`와 표준 출력 드라이버가 동시에 같은 출력 자원을 잡으면 콘솔 명령이
먹지 않는 상태가 생길 수 있기 때문입니다.

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
- PWM 출력 주파수: `IIM42652.hpp`의 `MOTOR_PWM_RATE`
- 텔레메트리 MAVLink 스트림: `src/modules/mavlink/streams/RT_CONTROL_TELEMETRY.hpp`

## 출력 매핑

- BLDC PWM 출력: 채널 1-4

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

수신 PC에서는 아래 두 방식 중 하나를 씁니다.

- PX4가 보내는 RT telemetry만 저장: `telem_csv_logger.py`
- OptiTrack UDP를 받아 PX4로 `ODOMETRY`를 보내고, PX4가 다시 보내는 RT telemetry까지 같은 프로세스에서 저장: `udp_pose_to_px4.py`

UDP 브리지 모드 예시:

```bash
python3 Tools/telem_csv_logger.py --mode udp --udp-port 14550 --output logs/rt_telem.csv
```

직렬 직접 수신 예시:

```bash
python3 Tools/telem_csv_logger.py --mode serial --serial-device /dev/ttyUSB0 --serial-baud 921600 --output logs/rt_telem.csv
```

OptiTrack UDP `x,y,z,roll,pitch,yaw`를 PX4로 보내고 RT telemetry도 같이 저장하는 예시:

```bash
python3 Tools/udp_pose_to_px4.py
```

이 스크립트는 다음을 한 번에 처리합니다.

- UDP `<6f>` 패킷 수신 (`x,y,z,roll,pitch,yaw`)
- MAVLink `ODOMETRY`로 PX4 송신
- PX4가 다시 보내는 `RT_CONTROL_TELEMETRY` 수신
- CSV 저장

중요:

- 기본값은 다음과 같습니다.
  - UDP 입력: `0.0.0.0:38030`
  - 각도 단위: `rad`
  - MAVLink 링크: UDP `192.168.2.1:14550`
  - serial 장치 자동 탐지는 `--mode serial`일 때만 사용
  - CSV 저장: `/home/psh/PX4-Autopilot/logs/rt_telem.csv`
- OptiTrack PC가 이미 `m`와 `rad` 기준으로 보내면 추가 옵션 없이 그대로 쓰면 됩니다.
- 장치나 포트를 강제로 바꾸고 싶을 때만 `--udp-host`, `--serial-device`, `--listen-port`, `--angle-unit` 같은 옵션을 주면 됩니다.
- 같은 serial 포트에서는 보통 `udp_pose_to_px4.py`와 `telem_csv_logger.py`를 동시에 실행하지 않는 편이 안전합니다.

생성된 CSV는 MATLAB `readtable()` 또는 Python `pandas.read_csv()`로 바로 읽을 수 있습니다.

DroneBridge 설정은 아래처럼 두는 것이 맞습니다.

- `UART serial protocol`: `MAVLink`
- `UART baud`: `921600`
- UDP 수신 포트: 보통 `14550`

## QGC USB 연결 주의

현재 RT telemetry는 `MAVLINK_MODE_ONBOARD`에서 `200 Hz`로 활성화되지만, 이 저장소에서는
USB CDC 링크에는 해당 스트림을 자동으로 붙이지 않도록 해 두었습니다. 즉:

- TELEM1 같은 companion 링크는 기존처럼 `onboard` 모드에서 RT telemetry 사용
- USB로 연결한 QGroundControl은 RT telemetry TUNNEL 없이 일반 MAVLink 위주로 동작

만약 USB에서 여전히 연결이 불안정하면 아래 설정도 같이 확인합니다.

```bash
param show USB_MAV_MODE
param set USB_MAV_MODE 5
param save
reboot
```

의미:

- `USB_MAV_MODE 5`는 USB 링크를 `config` 프로파일로 사용
- QGroundControl로 설정/로그 확인만 할 때는 이 값이 더 무난할 수 있음

## RT 텔레메트리 CSV 저장

수신기는 아래 스크립트 하나로 충분합니다.

```bash
python3 Tools/telem_csv_logger.py --mode udp --udp-port 14550 --output logs/rt_telem.csv
```

중요:

- `--output logs/rt_telem.csv`는 현재 셸 작업 디렉터리 기준 상대경로입니다.
- 예를 들어 `/home/psh`에서 실행하면 `/home/psh/logs/rt_telem.csv`에 저장됩니다.
- 저장 위치를 헷갈리지 않으려면 절대경로를 쓰는 편이 안전합니다.

예시:

```bash
python3 /home/psh/PX4-Autopilot/Tools/telem_csv_logger.py \
  --mode udp \
  --udp-port 14550 \
  --output /home/psh/PX4-Autopilot/logs/rt_telem.csv
```

CSV 컬럼:

- `cycle`: RT 루프 cycle 카운터
- `actual_start_s`: RT 루프 시작 이후 경과 시간 [s]
- `exec_us`, `input_us`, `control_us`, `output_us`
- `missed_cycles`
- `accel_x/y/z`, `gyro_x/y/z`
- `motor_1..motor_4`, `pwm_1..pwm_4`
- `opti_x/y/z`, `opti_roll/pitch/yaw`
- `opti_seq`, `opti_age_us`, `opti_valid`

MATLAB/Python 예시:

```matlab
T = readtable("/home/psh/PX4-Autopilot/logs/rt_telem.csv");
```

```python
import pandas as pd
df = pd.read_csv("/home/psh/PX4-Autopilot/logs/rt_telem.csv")
```

## 수신 확인 방법

보드 쪽:

```bash
iim42652 status
listener rt_control_telemetry 1
mavlink status
```

정상일 때 확인 포인트:

- `iim42652 status`에서 `RT telem topic=rt_control_telemetry advertised=true`
- `iim42652 status`에서 `pub_ok`가 증가
- `iim42652 status`에서 `pub_fail`는 증가하지 않거나 매우 작음
- `listener rt_control_telemetry 1`에서 토픽 1개가 출력됨
- `mavlink status`에서 `transport protocol: serial (/dev/ttyS5 @921600)`

PC 쪽:

- `telem_csv_logger.py` 실행 후 `rows=... last_cycle=...`가 계속 증가하면 수신 중입니다.
- 파일 크기가 계속 커지면 CSV 저장도 정상입니다.

## 주파수 해석

RT 루프 자체는 `CONTROL_PERIOD_US = 5000`이므로 내부 생성 주기는 200 Hz입니다.
다만 Wi-Fi/UDP/MAVLink 구간에서 몇 개 행이 빠질 수 있어서, 최종 CSV 저장 속도는
200 Hz보다 조금 낮게 보일 수 있습니다.

즉 아래 둘은 다른 값입니다.

- FC 내부 RT 생성 속도: 보통 200 Hz
- PC에 실제 저장된 CSV 행 속도: 링크 상태에 따라 약간 낮을 수 있음

확인 기준:

- `cycle`이 일정하게 증가하면 FC 내부 생성은 정상
- `actual_start_s` 차이가 평균 0.005 s 근처면 200 Hz에 가깝게 저장되고 있는 것
- `cycle`이 중간에 2 이상 건너뛰면 그만큼 중간 패킷이 빠진 것

간단 확인 예시:

```bash
python3 - <<'PY'
import pandas as pd
df = pd.read_csv('/home/psh/PX4-Autopilot/logs/rt_telem.csv')
dt = df['actual_start_s'].diff().dropna()
print('rows =', len(df))
print('saved_hz =', 1.0 / dt.mean())
print('cycle_loss =', (df['cycle'].diff().fillna(1) != 1).sum())
PY
```

해석:

- `saved_hz`가 200에 가까우면 거의 그대로 저장된 것
- `cycle_loss`가 0이면 CSV에 빠진 주기가 없다는 뜻
- `cycle_loss`가 0보다 크면 Wi-Fi/UDP/MAVLink/PC 수신 구간에서 일부 누락이 있다는 뜻

## 현재 구조의 의미

지금 구조는 다음과 같습니다.

- RT 루프 안에서는 센서 읽기, 제어 계산, 출력 계산만 수행
- RT 텔레메트리는 RT 루프에서 lock-free로 큐잉
- uORB publish와 MAVLink 송신은 RT 루프 바깥 문맥에서 수행
- 따라서 RT 제어 주기를 직접 막지 않도록 구성되어 있음

즉 CSV에 저장된 값은 RT 루프에서 만든 값을 바깥으로 꺼내서 기록한 것이고,
실제 제어 루프와 완전히 분리된 경로로 나갑니다.

## ESC High-Low Calibration

MAVLink Console 또는 NSH 셸에서 아래 명령만 사용합니다.

```bash
iim42652 esc_calib status
iim42652 esc_calib high
iim42652 esc_calib low
iim42652 esc_test 3
```

의미:

- `status`: 현재 ESC calibration 출력 상태 확인
- `high`: PWM 출력 1-4를 `2000 us`로 고정
- `low`: PWM 출력 1-4를 `1000 us`로 고정
- `esc_test <0..10>`: PWM 출력 1-4를 제한된 저출력으로 고정해 회전 여부만 확인

아래 순서는 `FC를 USB 등으로 먼저 켜고`, 그 다음에 `ESC 메인 전원`을 넣을 수 있을 때만 성립합니다.
즉 FC와 ESC가 같은 배터리에서 동시에 올라오는 배선이라면, 소프트웨어로 `high`를 먼저 넣는 방식의
High-Low calibration은 할 수 없습니다.

분리 전원 구성이 가능할 때의 순서:

```bash
# 1) FC 전원 인가 후 high 출력 준비
iim42652 esc_calib high

# 2) 그 상태에서 ESC 쪽 전원 인가
# 3) 삐~삐~ 소리 뒤 low로 전환
iim42652 esc_calib low

# 4) ESC 전원 off
```

현재는 이전 실험용 모터 구동 경로를 제거했고, 임의 가변 출력 명령은 없습니다.
다만 처음 연결 확인을 위해 `iim42652 esc_test <0..10>`만 제한적으로 남겨 두었습니다.

같은 배터리로 FC와 ESC가 동시에 켜지는 경우:

- `iim42652 esc_calib high`를 먼저 넣어도 ESC는 이미 전원이 들어온 뒤라 calibration 시작 조건을 놓칩니다.
- 이 경우 처음 연결 확인은 `low(1000 us)` 유지 상태에서 ESC 부팅음과 모터 정지만 확인합니다.
- High-Low calibration이 꼭 필요하면 FC를 USB 또는 별도 5V로 먼저 켠 뒤 ESC 메인 전원을 나중에 넣을 수 있어야 합니다.

## 처음 ESC/모터 연결 후 테스트

처음 배선했을 때는 반드시 프로펠러를 제거한 상태에서 확인합니다.

현재 펌웨어에서는 실시간 제어기 출력이 실제 모터 PWM으로 전달되지 않으므로,
처음 연결 후 테스트 목적은 아래 세 가지만 확인하는 것입니다.

- 평상시 PWM `1000 us`가 정상적으로 유지되는지
- 필요할 때만 High-Low calibration이 되는지
- 제한된 저출력에서만 짧게 회전 확인이 되는지

권장 순서:

```bash
# 1) FC만 먼저 켜기
# USB 또는 FC 전원만 인가하고 ESC 메인 전원은 아직 넣지 않음

# 2) 현재 출력 상태 확인
iim42652 status
iim42652 esc_calib status
```

이때 확인할 점:

- `iim42652 esc_calib status`가 `low` 또는 idle 상태여야 함
- `iim42652 status`의 `pwm_us`가 4채널 모두 `1000 us`여야 함
- 이 상태는 MR-X4 기준 최소 스로틀, 즉 `0%` 명령 상태임

다음으로 ESC와 모터를 처음 연결한 상태를 확인합니다.

```bash
# 3) low 상태를 한 번 더 명시
iim42652 esc_calib low

# 4) 그 다음 ESC 메인 전원 인가
```

정상이라면:

- ESC 초기음만 나고 모터는 돌지 않아야 함
- `1000 us`가 유지되는 동안 모터가 계속 돌면 배선, ESC 설정, calibration 상태를 다시 확인해야 함

저출력 회전 확인이 필요하면 아래처럼 아주 작게만 확인합니다.

```bash
# 5) 프로펠러 제거 상태에서만 수행
iim42652 esc_test 3

# 모터가 안 돌면 필요할 때만 5, 8, 10 순서로 아주 짧게 확인
# 예: iim42652 esc_test 5

# 6) 확인이 끝나면 즉시 low로 복귀
iim42652 esc_calib low
```

권장:

- 처음에는 `3%`부터 시작
- 꼭 필요할 때만 `5%`, `8%`, `10%` 순으로 올림
- `10%`를 넘는 테스트는 현재 펌웨어에서 허용하지 않음
- 회전 확인은 아주 짧게만 하고 바로 `low`로 내림

High-Low calibration이 필요하면 아래 순서로 진행합니다.
이 절차는 FC와 ESC 전원을 분리해서 순서를 만들 수 있을 때만 가능합니다.

```bash
# 5) ESC 전원을 끈 상태에서 high 준비
iim42652 esc_calib high

# 6) ESC 메인 전원 인가
# 7) 삐~삐~ 소리 뒤 low로 전환
iim42652 esc_calib low

# 8) ESC 전원 off
```

Calibration 후 재확인:

- ESC 전원을 다시 넣었을 때 모터가 자동으로 돌지 않아야 함
- `iim42652 status`에서 `pwm_us`는 다시 `1000 us` 4채널로 보여야 함

중요:

- `esc_test`는 회전 확인용으로만 남겨 둔 제한 명령입니다.
- 현재 펌웨어에서 `esc_test`는 `0..10%` 범위만 허용합니다.
- 실제 추력 시험이나 연속 구동은 별도 제어 경로를 다시 넣기 전까지 수행하지 않습니다.

## 부팅 후 확인

NSH 셸에서 아래 명령으로 확인합니다.

```bash
iim42652 status
```

출력에서 보게 되는 주요 항목:

- `RT period_us=5000`
- `output_mode`
- `RT telem topic=rt_control_telemetry advertised=true`
- cycle 카운터
- input/control/output/exec 시간
- 가속도/자이로 값
- PWM 출력 값

상태 확인 팁:

- ESC calibration 상태 확인은 `iim42652 esc_calib status`가 가장 직접적입니다.
- `iim42652 status`의 `output_mode`, `pwm_us`도 같이 볼 수 있습니다.

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
