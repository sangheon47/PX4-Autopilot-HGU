# PX4-Autopilot-HGU

이 저장소는 `CUAV 7-Nano + IIM42652` 직접 실시간 루프 실험을 위해 정리한
PX4 포크입니다.

현재 이 저장소에서는 아래 2개 브랜치 구조를 기준으로 사용합니다.

- `team`: 팀이 함께 쓰는 메인 작업 브랜치
- `share`: 필요할 때만 남겨두는 보존용 기준 브랜치

즉 평소 작업은 거의 전부 `team`에서 하고, `share`는 꼭 필요할 때만 기준점으로
남겨두는 방식입니다.

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

부팅 후 확인:

```bash
iim42652 status
```

## 현재 출력 구조

- `iim42652`는 부팅 시 `boards/cuav/7-nano/init/rc.board_sensors`에서 자동 시작됩니다.
- `iim42652` 드라이버가 Servo 출력 1-4와 BLDC DShot 출력 5-6을 직접 초기화하고 제어합니다.
- `ROMFS/px4fmu_common/init.d/rcS`의 표준 `dshot start`는 출력 소유권 충돌을 막기 위해 비활성화되어 있습니다.
- BLDC는 기본 모드가 `stop`이며, 드라이버 시작 직후 약 3초 동안 startup hold가 걸립니다.

## MAVLink Console 수동 BLDC 제어

MAVLink Console 또는 NSH 셸에서 아래 명령으로 BLDC(DShot, 채널 5-6)를 수동 제어할 수 있습니다.

```bash
iim42652 motor status
iim42652 motor stop
iim42652 motor set 0.03
iim42652 motor set 0.05
iim42652 motor set 0.10
iim42652 motor auto
```

의미:

- `iim42652 motor status`: 현재 모드와 수동 설정값 확인
- `iim42652 motor stop`: BLDC 강제 정지
- `iim42652 motor set <0..1>`: BLDC를 지정 출력으로 고정
- `iim42652 motor auto`: `rt_controller()` 출력으로 복귀

권장 시험 순서:

```bash
iim42652 motor status
iim42652 motor set 0.03
# 필요하면 0.05, 0.10 등으로 증가
iim42652 motor stop
```

추가 메모:

- `auto`는 `rt_controller()`가 만든 값을 그대로 사용합니다.
- 현재 `src/lib/rt_control/rt_control.c`에서는 BLDC 출력이 `0.5`, `0.5`로 고정되어 있어 `auto`는 사실상 50% 고정 출력처럼 동작합니다.
- `iim42652 status`의 `manual_mode`, `manual`, `bldc_dshot`는 참고할 수 있지만, BLDC 퍼센트 표시는 현재 최종 override 출력과 완전히 일치하지 않을 수 있습니다.

## 자세한 문서

실시간 루프 구조, 수정 포인트, 협업 방식, 다른 팀이 fork해서 쓰는 구조는 아래
문서를 보면 됩니다.

- [README_HGU.md](README_HGU.md)

## 주요 파일

- `boards/cuav/7-nano/default.px4board`
- `boards/cuav/7-nano/init/rc.board_sensors`
- `ROMFS/px4fmu_common/init.d/rcS`
- `src/drivers/imu/invensense/iim42652/IIM42652.cpp`
- `src/drivers/imu/invensense/iim42652/IIM42652.hpp`
- `src/lib/rt_control/rt_control.c`
- `src/lib/rt_control/rt_control.h`

## 참고

- 이 저장소는 공식 PX4를 기반으로 한 포크입니다.
- 공식 PX4 원본 저장소: <https://github.com/PX4/PX4-Autopilot>
- 공식 PX4 문서: <https://docs.px4.io/main/en/>
