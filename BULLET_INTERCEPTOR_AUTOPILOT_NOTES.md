# Bullet Interceptor Autopilot Notes

이 문서는 현재 기체 형상과 연구 방향을 기준으로, `무엇을 먼저 만들어야 하는지`를
짧고 명확하게 정리한 메모다.

현재 전제:

- 기체는 `quad tailsitter` 계열이다.
- 최종 목표는 `PX4 사용`이 아니라 `내 autopilot 설계`다.
- 교수님 관점은 `정지 호버 유지`보다 `발사 후 바로 틸팅해서 전진비행`에 가깝다.
- 센서는 우선 `IMU + barometer`만 있다고 가정한다.

## 1. 용어 정리

### 상태기계

`상태기계(state machine)`는 지금 기체가 어떤 비행 단계에 있는지 나누는 상위 로직이다.

예:

- `IDLE`
- `BOOST`
- `PITCH_OVER`
- `CRUISE`

즉 상태기계는 "`지금 어떤 제어 목표를 써야 하는가`"를 결정한다.

### 상태공간 벡터

`상태공간 벡터(state vector)`는 현재 기체의 물리 상태를 수학적으로 적는 것이다.

예:

```text
x = [q_att, p, q, r, z, z_dot]^T
```

또는

```text
x = [phi, theta, psi, p, q, r, z, z_dot]^T
```

즉 상태공간은 "`기체가 지금 어떤 상태인가`"를 표현한다.

### 둘의 차이

- 상태기계: 비행 모드 전환용
- 상태공간 벡터: 추정기와 제어기 계산용

연구에서는 둘 다 필요할 가능성이 높다.

## 2. 교수님 말의 의미

교수님이 "`호버링 제어기는 굳이 필요 없다`"고 하신 말은, 보통 아래 뜻으로 해석하는 것이 맞다.

- `정지 호버 유지용 outer loop`는 핵심이 아니다.
- 발사 후 바로 전진비행 전환이 목적이다.
- 하지만 `자세 안정화`와 `각속도 감쇠`는 여전히 필요하다.

즉 없어도 되는 것은:

- 장시간 `position hold`
- 장시간 `hover hold`
- 정교한 저속 제자리 유지

그래도 필요한 것은:

- 발사 직후 자세 붕괴 방지
- pitch-over 도중 각속도 안정화
- 최소 고도 바닥 보호

한 줄로 정리하면:

`호버 제어기`는 없어도 될 수 있지만, `안정화 제어기`는 없어지지 않는다.

## 3. 지금 목표로 삼을 최소 autopilot

현재 단계에서 목표는 완전한 VTOL autopilot이 아니라 아래 정도가 적절하다.

- 발사 직후 기체가 즉시 뒤집히지 않음
- 일정 시간 또는 고도 조건 후 기체가 틸팅함
- 틸팅 중 각속도가 제한됨
- 최소 고도 이하로 떨어지지 않도록 thrust 또는 tilt를 제한함

즉 목표는:

`boost -> tilt -> forward-flight entry`

이지,

`takeoff -> stable hover -> position hold -> landing`

가 아니다.

## 4. 추천 상태기계

### `IDLE`

목적:

- 대기
- 센서 초기화
- launch 준비

입력:

- IMU
- barometer
- arm / launch command

출력:

- 모터 정지 또는 idle

전이:

- launch 명령 수신 시 `BOOST`

### `BOOST`

목적:

- 짧은 시간 동안 충분한 추력 확보
- 초기 자세 붕괴 방지

입력:

- gyro
- accel
- barometer

제어:

- collective thrust는 시간 기반 또는 schedule 기반
- `p, q, r` rate damping
- 필요하면 launch 기준 자세 유지

전이:

- `t > T_boost`
- 또는 `z > z_min_for_pitch_over`

둘 중 하나를 만족하면 `PITCH_OVER`

### `PITCH_OVER`

목적:

- 기체를 수직 launch 자세에서 전진비행 자세로 전환

입력:

- gyro
- accel
- barometer

제어:

- 목표 자세 `q_ref(t)` 또는 목표 pitch schedule 추종
- rate loop는 계속 유지
- 고도가 너무 빠르게 떨어지면 tilt 속도 제한

전이:

- 목표 자세 근처 도달
- 또는 목표 전환 시간 경과

그러면 `CRUISE`

### `CRUISE`

목적:

- 전진비행 자세 유지
- 이후 guidance / navigation 실험 확장 가능

입력:

- IMU
- barometer
- 나중에는 airspeed 또는 외부 속도 추정 추가 가능

제어:

- attitude hold 또는 trimmed attitude hold
- thrust schedule

### `ABORT` 또는 `FAILSAFE`

초기 연구여도 두는 게 좋다.

목적:

- 센서 이상
- 각속도 과대
- 최소 고도 미만

동작:

- thrust cut 또는 안전한 fallback 로직

## 5. 최소 상태공간 벡터

초기 연구용으로는 너무 큰 모델보다 작은 모델이 낫다.

추천 시작점:

```text
x = [q_att, p, q, r, z, z_dot]^T
u = [T, tau_x, tau_y, tau_z]^T
```

또는 Euler 기반 단순화:

```text
x = [phi, theta, psi, p, q, r, z, z_dot]^T
u = [T, tau_phi, tau_theta, tau_psi]^T
```

주의:

- tailsitter에서는 `Euler 0`이 항상 "원하는 자세"를 의미하지 않는다.
- 그래서 장기적으로는 Euler보다 quaternion 기준이 덜 헷갈린다.
- 특히 `launch 자세`, `transition 목표 자세`, `cruise 자세`를 quaternion으로 두는 편이 낫다.

## 6. IMU + barometer만으로 가능한 것

가능:

- 각속도 안정화
- roll / pitch 추정
- 짧은 구간의 자세 유지
- 상대 고도 추정
- 최소 고도 보호

애매하거나 어려움:

- 절대 yaw heading 유지
- x/y 위치 유지
- 정확한 저고도 terrain following
- 전진비행 속도 정확 추정

즉 현재 센서만으로도

`발사 -> 틸팅 -> 전진진입`

연구는 충분히 시작할 수 있다.

## 7. 지금 바로 만들 제어 구조

### 1단계: rate loop

가장 먼저 만들 것:

```text
[p_ref, q_ref, r_ref] - [p, q, r] -> rate controller -> desired torque
```

이 단계 목표:

- 기체가 launch 직후 바로 발산하지 않게 만들기

### 2단계: attitude command

그다음:

```text
attitude reference -> attitude error -> desired body rates -> rate controller
```

여기서 attitude reference는

- launch 자세
- pitch-over 중간 자세
- cruise 자세

로 바뀐다.

### 3단계: thrust / altitude floor

그다음:

- thrust를 시간 기반 schedule로 넣기
- baro로 최소 고도만 보호

즉 "`고도 정확히 유지`"보다 "`바닥에 꽂히지 않게 하기`"가 먼저다.

## 8. 연구 관점에서 권장하는 다음 순서

### Step 1. 시뮬레이션에서 launch profile 고정

먼저 문장으로 정한다.

예:

1. launch command
2. 1.0~1.5초 boost
3. 0.8~1.2초 동안 pitch-over
4. cruise attitude 진입

### Step 2. open-loop로 먼저 돌려보기

아직 제어기 없이도 아래를 먼저 볼 수 있다.

- thrust schedule
- pitch schedule

이걸로 기체가 어떻게 무너지는지 봐야 한다.

이 단계에서 얻는 것:

- 필요한 최소 thrust 대략치
- pitch-over를 얼마나 빨리 하면 망하는지

### Step 3. rate damping 추가

open-loop에서 망가지는 축을 기준으로 `p, q, r` damping을 넣는다.

이 단계 목표:

- launch와 tilt 과정에서 급격한 각속도 발산 억제

### Step 4. attitude tracking 추가

launch 자세에서 cruise 자세로 가는 기준 자세를 만든다.

예:

- 시간 기반 quaternion interpolation
- 또는 pitch schedule 기반 reference

### Step 5. altitude floor 추가

barometer를 이용해 아래 조건을 건다.

- `z < z_floor`이면 pitch-over 중지 또는 완화
- 또는 collective thrust 증가

이건 착륙 제어가 아니라 `지면 충돌 방지용 최소 안전장치`다.

## 9. 시뮬레이션에서 당장 해야 할 것

### 가장 먼저

- 모델 축과 모터 방향 재확인
- thrust 증가 시 기체가 어느 축으로 반응하는지 확인
- pitch-over를 어느 축으로 줄 것인지 고정

### 그다음

- `open-loop thrust + pitch schedule` 실험
- 로그 저장
- rate damping controller 추가

### 그 이후

- attitude tracking
- altitude floor

## 10. 현재 저장소 기준으로 연결되는 파일

- 시뮬레이션 모델: [Tools/simulation/gz/models/bullet_interceptor/model.sdf](/home/psh/PX4-Autopilot/Tools/simulation/gz/models/bullet_interceptor/model.sdf:1)
- 시뮬레이션 airframe: [ROMFS/px4fmu_common/init.d-posix/airframes/22000_gz_bullet_interceptor](/home/psh/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/22000_gz_bullet_interceptor:1)
- 실기 RT 루프: [src/drivers/imu/invensense/iim42652/IIM42652.cpp](/home/psh/PX4-Autopilot/src/drivers/imu/invensense/iim42652/IIM42652.cpp:1)
- 제어기 코드 자리: [src/lib/rt_control/rt_control.c](/home/psh/PX4-Autopilot/src/lib/rt_control/rt_control.c:1)

## 11. 결론

현재 네가 다음으로 해야 할 일은 `호버 제어기 완성`이 아니다.

현재 우선순위는 아래다.

1. launch / transition / cruise 상태기계 정의
2. open-loop thrust + tilt schedule 확인
3. rate stabilization 추가
4. attitude tracking 추가
5. baro 기반 altitude floor 추가

즉,

`hover hold`

가 아니라

`boost-transition autopilot`

을 먼저 만든다고 생각하면 된다.
