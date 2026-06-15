# flight_safety

실내 자율비행 안전 모니터링. 현재 **M1: observe-only** (감지·보고만, 개입 없음).

설계: [`docs/flight_safety_architecture.md`](docs/flight_safety_architecture.md)

## 구성 요소

- **`TopicMonitor`** (general) — `(topic, checks, thresholds)`만 주면 감지:
  - `dropout` — 마지막 메시지 이후 timeout 초과 (순간 공백 / liveness, ERROR)
  - `rate` — 평균 Hz가 min_rate 미만 (만성 저throughput, WARN)
  - `freeze` — 값(위치)이 freeze_window 동안 불변 (occlusion hold, ERROR)
  - `jump` — 연속 두 샘플의 위치 거리가 임계 초과 (마커 스왑/텔레포트, WARN)
  - `nan` — NaN/inf 카운트
  - dropout/rate는 **임의 토픽**(AnyMsg, 타입 불필요). freeze/jump/nan만 `msg_type` 필요.
- **`PingMonitor`** (general) — host 도달성/지연 (LINK 레이어).
- **`PairConsistencyMonitor`** — 두 pose 스트림 일치도(VRPN vs `/mavros/local_position`).
  **그냥 같아야 함** — raw 위치 오차를 직접 비교, 차이 자체를 fault로 경보 (offset 보정 없음).

모니터 추가 = `config/monitors.yaml`의 `subsystems: <이름>:` 아래 항목. 코드 변경 없음.

## 실행

catkin ws에 넣고(또는 이 디렉터리를 `<ws>/src`에 심볼릭링크) 빌드:

```bash
catkin build flight_safety        # 또는 catkin_make
source <ws>/devel/setup.bash
roslaunch flight_safety monitoring.launch
```

보기:

```bash
rosrun rqt_robot_monitor rqt_robot_monitor   # 트리 뷰
# 또는
rostopic echo /diagnostics
```

## 읽는 법 (consistency)

VRPN과 `/mavros/local_position`은 **같은 프레임/원점이라 그냥 같아야** 한다. 차이 자체가 잡으려는 fault.

- `err_m` > 임계 → **불일치 = fault** (프레임 오프셋이든 발산이든 전부 잡음).
- `err_xyz_m` → 어느 축이 어긋났는지 (프레임 정렬 디버깅에 유용).
- 빠른 기동 중 작은 일시 오차는 EKF2 필터 lag → 임계가 그만큼만 허용.

## M2 반응 (`safety_node.py`, ⚠️ KILL 권한)

`monitor_node`(observe-only)와 **분리**. 무장 상태에서만 개입(`require_armed`), `react_rate_hz` event-driven.

**응답 3종** (severity hold<land<kill):
| 응답 | 동작 |
|---|---|
| `hold` | 활성 시점 위치 잠가서 유지 (offboard position setpoint) |
| `land` | 제자리 하강 (offboard velocity, vz) |
| `kill` | force-disarm (terminal) |

**fault → response 규칙** (`rules:`, 추가/수정은 config 한 줄):
| fault | 응답 |
|---|---|
| `vrpn/stream` ERROR (dropout/freeze/rate) | land |
| `vrpn/consistency` ERROR (local_position 불일치) | land |
| `rc/stream` ERROR (RC 안나옴) | land |
| geofence **이탈** (VRPN 판별) | kill |
| geofence **접근** | clamp (MUX에서, kill/land 아님) |
| geofence VRPN 모름 (stale) | 무동작 (VRPN fault가 처리) |

**control 우선순위 사다리** (supervisor `_tick`에 명시, 4 > 3 > 2 > 1):
- **4 KILL**: terminal(geofence 이탈 등) → force-disarm. **파일럿보다도 위** — 수동조종 중에도 geofence 이탈 시 kill (하드 geofence)
- 3 Manual: OFFBOARD 중 RC 스틱 편향 → POSCTL 전환, 발행 중단 (파일럿 이양)
- 2 Response: 활성 응답(hold/land) setpoint
- 1 Normal: `normal_in`(JAX/control_bridge) passthrough
- 2/1은 출력 1곳에서 geofence clamp 적용 → `setpoint_out`

⚠️ 통합 필요: **control_bridge가 `/flight_safety/normal_setpoint`로 발행하도록 변경**해야 MUX가 정상 경로가 됨. 또 setpoint **frame/sign 미검증**(VERIFY-BEFORE-FLIGHT V1).

```bash
roslaunch flight_safety safety.launch    # KILL 권한 — 의도적으로 실행할 때만
```
