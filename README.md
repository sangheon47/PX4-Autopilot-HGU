# PX4-Autopilot-HGU

이 저장소는 `CUAV 7-Nano + IIM42652` 직접 실시간 루프 실험을 위해 정리한
PX4 포크입니다.

기본 브랜치는 `share`이며, 현재 이 저장소에서는 아래 3개 브랜치 구조를
기준으로 사용합니다.

- `share`: 외부 공유 및 재현용 기준 브랜치
- `team`: 팀 통합 브랜치
- `local`: 개인 작업 브랜치

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

부팅 후 확인:

```bash
iim42652 status
```

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
