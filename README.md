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
git switch team
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
