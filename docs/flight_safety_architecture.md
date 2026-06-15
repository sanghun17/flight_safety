# 실내 자율비행 안전 아키텍처

> 상태: **설계(DRAFT)** — 구현 전. "DO NOT FLY" 단계.
> 대상: mocap(OptiTrack/VRPN) 기반 실내 PX4 + MAVROS 드론, JAX MPPI offboard 제어.
> 범위: 추정·안전·제어 계층의 구조 설계. 코드 아님.

---

## 0. 한 줄 요약

VRPN mocap을 1차 위치원으로 쓰되, **mocap 상실 시 학습형 관성 오도메트리(AirIO)가 body velocity를 공급**해 통제된 비상 하강을 가능케 한다. 안전 판단은 **companion 안전 모니터(L1)** 가 fault별로 등급화해 수행하고, geofence는 **control_bridge의 속도 클램프(L2)** 로 강제한다.

---

## 1. 설계 원칙 & 가정

### 1.1 원칙
- **계층 방어**: 단일 노드/센서 고장이 곧 추락이 되지 않도록 독립적인 방어선을 쌓는다.
- **fault별 개별 대응**: "비상=무조건 LAND"가 아니라, 각 fault의 추정 영향에 맞춰 대응을 다르게 한다(§5).
- **추정 신뢰도와 행동의 결합**: 위치 추정이 망가진 상태에서 위치 기반 동작(HOLD/geofence)은 위험하므로, 행동은 항상 추정 health에 게이트된다.
- **degrade-don't-fail**: mocap → 학습형 관성 → 자세유지 하강 순으로 *우아하게 열화*한다.

### 1.2 가정 (load-bearing — 틀리면 설계가 무너지는 전제)
- **(A1) Companion PC(Jetson)는 비행 중 죽지 않는다.** ← 운영 결정. 이 가정 하에 firmware 독립 failsafe(L0)를 1차 방어선에서 제외한다. **잔존 위험은 §10에 등록.**
- **(A2) 세이프티 파일럿이 RC와 킬스위치를 상시 들고 대기한다.** 단 미숙/지연 가능 → 소프트웨어가 1차로 막고 RC는 물리적 최후 수단.
- **(A3) mocap 캡처 볼륨 안에서만 비행한다.** geofence 박스 ⊆ 캡처 볼륨.
- **(A4) AirIO는 본 드론·본 IMU로 학습된다.** pretrained 가중치 그대로 쓰지 않는다(§7).

---

## 2. 시스템 토폴로지

```
  ┌─ 추정(Estimation) ─────────────────────────────────────────────┐
  │                                                                 │
  │  [OptiTrack Motive] ──VRPN 100Hz──▶ [vrpn_client_node]          │
  │                                          │ /vrpn.../pose        │
  │                                          ▼                      │
  │  [FCU IMU 200Hz] ──▶ [AirIO node] ──┐  [vision_pose MUX +       │
  │   /mavros/imu/data    (body vel+cov) │   health/freeze/jump]    │
  │                                      └──▶│                      │
  │                                          │ VRPN ok → vision_pose/pose (position)
  │                                          │ VRPN bad & AirIO ok → vision_speed/speed_body (velocity)
  │                                          ▼                      │
  │                                     [MAVROS] ──▶ [PX4 EKF2] ──▶ /mavros/local_position/*
  └─────────────────────────────────────────────────────────┬──────┘
                                                             │ local_position
  ┌─ 계획·제어(Planning/Control) ──────────────────────────────┼──────┐
  │                                                          ▼      │
  │ [fast-livo /aft_mapped_to_init] ─▶ /robot/odom ─▶ [exploration  │
  │   (FCU와 무관, planner 전용)                        planner]     │
  │                                                       │         │
  │                                                       ▼         │
  │                                                  [JAX MPPI] ─▶ /jax/optimal_trajectory
  │                                                       │         │
  │  소프트웨어 명령원 (우선순위 高→低):                    │         │
  │   ① 안전모니터 emergency (brake/descend/hold)          │         │
  │   ② operator companion-teleop (선택)                  │         │
  │   ③ JAX/planner 정상 ◀──────────────────────────────────┘        │
  │   ④ idle zero-hold (기본)                                        │
  │        │ (모든 명령원)                                           │
  │        ▼                                                         │
  │   [Command MUX]  우선순위 선택 (twist_mux 패턴 + lock 토픽)       │
  │        │ 선택된 단일 명령                                         │
  │        ▼                                                         │
  │   [L2 단일 CBF geofence 클램프]  (local_position, health-gated)   │
  │        │                                                         │
  │        ▼                                                         │
  │   [mode guard]  /mavros/state == OFFBOARD 일 때만 발행            │
  │        │ /mavros/setpoint_raw/local (vel, NED)                   │
  │        ▼                                                         │
  │     [PX4] ─▶ motors                                              │
  │        ▲                                                         │
  │   COM_RC_OVERRIDE (firmware): RC 스틱 ▶ OFFBOARD 이탈 ▶ 수동      │
  └────────┼──────────────────────────────────────────────────────────┘
           │ RC
    [세이프티 파일럿]

  ┌─ 안전(Safety) ─────────────────────────────────────────────────┐
  │ [L1 Safety Monitor] 감시: VRPN health, AirIO cov, EKF2 status,  │
  │                          geofence, MAVROS heartbeat, 타임아웃    │
  │                     행동: MUX 전환 / control_bridge disable /     │
  │                          MAVROS set_mode(AUTO.LAND) / alert       │
  │ [L3 세이프티 파일럿 + RC 킬스위치]  (물리적, 소프트웨어 독립)      │
  └─────────────────────────────────────────────────────────────────┘
```

핵심 분리: **fast-livo는 planner/voxblox 전용이고 FCU(EKF2)에는 들어가지 않는다.** FCU의 위치원은 VRPN(1차) / AirIO(fallback) 뿐이다.

---

## 3. 추정 스택

### 3.1 vision_pose 단일화 (현재 충돌 해소)
현재 `optitrack.launch`(VRPN→vision_pose @30Hz)와 `real_flight.launch`의 `vision_pose_relay.py`(FAST-LIVO→vision_pose @50Hz)가 **같은 토픽을 동시에 노린다 → 충돌.** 이를 **단일 MUX 노드**로 대체한다:

- **입력**: VRPN pose, AirIO body velocity(+cov).
- **정책**:
  - VRPN healthy → `/mavros/vision_pose/pose`(position)로 VRPN 중계.
  - VRPN bad & AirIO healthy → `/mavros/vision_speed/speed_body`(velocity)로 AirIO 중계.
  - 둘 다 bad → 중계 중단(§5 종단 대응).
- **프레임**: VRPN(Motive 축) ↔ EKF2(ENU) 정렬을 MUX 진입 전에 1회 검증·고정(VERIFY-BEFORE-FLIGHT, §8).
- **출력은 position XOR velocity** — 동시에 둘 다 EKF2에 먹이지 않는다.

### 3.2 왜 fallback이 velocity인가
AirIO는 **절대 위치를 주지 못한다**(관성 적분은 절대 위치 관측 불가, 표류). 대신 학습으로 **body velocity를 회귀**한다(RTE@5s ≈ 0.8m, raw 적분 대비 자릿수 우수). 따라서:
- 표류하는 position을 vision_pose로 먹이면 EKF2가 곧 reject → 오히려 위험.
- velocity aiding으로 먹이면 **수평 속도가 bounded** → position은 천천히 표류하되 *통제된 짧은 하강(수 초)* 동안은 sub-meter. 이게 fallback의 정확한 역할이다.

### 3.3 EKF2 innovation gate
mocap jump/마커 스왑은 innovation 스파이크로 나타난다. EKF2의 vision innovation gate 파라미터로 **펌웨어 단에서 나쁜 샘플을 거부**하게 둔다(MUX의 jump 감지와 이중화). 해당 파라미터는 레포에서 버전관리한다(§8, 현재 0건).

---

## 4. 안전 계층

가정 A1(companion 불사)에 따라 **L1이 1차 방어선**이다. L0는 "거의 공짜 보험"으로 축소(§10).

| 계층 | 구현 | 역할 | 독립성 |
|---|---|---|---|
| **L1** | companion 안전 모니터 노드(신규, `odom_guard` 패턴) | fault 감지 → 등급별 대응(§5). MUX 전환, control disable, AUTO.LAND 트리거 | Jetson 위 (A1 전제) |
| **L2** | Command MUX(우선순위 중재) + 단일 CBF 속도 클램프 | 소프트웨어 명령원 중재 + offboard geofence 강제(§6) | Jetson 위 |
| **L0** | PX4 파라미터(EKF2 innovation gate, **RC경로 GF_***) | 컴패니언이 못 가로채는 **RC 입력 geofence** + 추정 위생 | **firmware (독립)** |
| **L3** | 세이프티 파일럿 + RC 킬스위치 | 물리적 최후 수단 | 완전 독립 |

> L0를 1차에서 뺐지만 **세 가지는 firmware에만 가능**해 최소한으로 유지한다: ① EKF2 innovation gate(나쁜 vision 거부), ② **RC 입력 geofence(`GF_*`)**, ③ **RC takeover(`COM_RC_OVERRIDE`)**. ②③은 companion이 RC 스틱을 가로챌 수 없어 firmware만 가능하고(§6.4), 셋 다 설정 비용이 거의 0이므로 A1과 무관하게 넣기를 권고.

---

## 5. Fault 분류 → 등급별 대응 매트릭스

| # | Fault | 감지 | 추정 영향 | 대응 |
|---|---|---|---|---|
| F1 | VRPN 짧은 gap (<~0.5s) | vision_pose stamp 간격 | EKF2 잠깐 IMU coast | **무대응**, 회복 대기 (정상) |
| F2 | VRPN freeze(stale, 값 동결) | 연속 동일 pose 감지 | **위험**: stale 융합 | F4로 격상 → AirIO 전환 |
| F3 | VRPN jump/텔레포트 | EKF2 innovation gate + MUX 값-점프 | **위험**: 추정 튐 | 해당 샘플 거부, 지속 시 F4 |
| F4 | **VRPN 지속 상실** (>timeout) | stamp timeout | 절대 위치 상실 | **MUX가 AirIO velocity로 전환** → 통제 하강(AUTO.LAND), geofence 박스 축소 |
| F5 | AirIO 불확실(cov 폭증) | AirIO 출력 cov 임계 | velocity 신뢰 불가 | 즉시 AUTO.LAND, 파일럿 alert |
| F6 | **VRPN + AirIO 동시 상실** | F4 ∧ F5 | 수평 blind | **최약 rung**: 자세유지 + baro 하강(AUTO.LAND) + 저고도 land detector disarm. 거친 하강 — §10 위험 |
| F7 | fast-livo 발산 | `odom_guard` (>100m) | **planner만** 영향, FCU 무관 | laserMapping kill, planner stop. FCU는 VRPN/AirIO로 계속 |
| F8 | geofence 위반(예측) | L2 CBF | — | 경계 법선 속도 클램프 → 경계서 정지 |
| F9 | EKF2 발산/innovation 지속 | `/mavros/estimator_status` | 제어 위험 | commands_enabled off + AUTO.LAND |
| F10 | MAVROS 끊김 | heartbeat timeout | 명령 전달 불가 | 파일럿 인계(offboard-loss는 본래 firmware 영역) |
| F11 | control_bridge/JAX stall | trajectory timeout | 명령 정지 | commands_enabled off → hover/하강 |
| F12 | 비행 타임아웃 | wall-clock | — | 자동 AUTO.LAND |

대응 동사 정의:
- **AUTO.LAND**: companion이 `mavros/set_mode` AUTO.LAND 트리거 → FCU의 baro 기반 하강 + land detector 컷오프. (firmware 모드지만 companion이 트리거 — A1 하에서 허용.)
- **geofence 박스 축소**: fallback 중에는 추정 신뢰도가 낮으므로 허용 영역을 보수적으로 좁힌다.
- **파일럿 takeover**: RC 스틱 임계 초과 시 `COM_RC_OVERRIDE`로 PX4가 OFFBOARD 이탈 → 수동. 모든 소프트웨어 대응의 *위에* 있는 최상위(§6.4), firmware가 강제(= 항상 가능, fault 아님).

---

## 6. 명령/제어 경로: Command MUX (L2)

estimation MUX(§3.1)와 대칭으로, 명령 측에도 **단일 Command MUX**를 둔다. 소프트웨어 명령원이 여럿(JAX/planner, 안전모니터 emergency, idle hold)이므로 우선순위로 중재해 **하나만** 고르고, **그 출력에 geofence CBF를 한 번만** 적용한 뒤 `/mavros/setpoint_raw/local`로 발행한다. 현재 `control_bridge`가 bounds 없이 직접 발행하던 구조를 **단일 chokepoint**로 대체한다.

### 6.1 우선순위 (소프트웨어 명령원, 高→低)
1. **안전모니터 emergency** — brake / descend / hold (L1 override).
2. **operator companion-teleop** — 선택(있으면).
3. **JAX/planner 정상** — 평시 자율.
4. **idle zero-hold** — 기본(아무 것도 활성 아니거나 비-OFFBOARD).

`twist_mux`의 우선순위 + lock 토픽 패턴을 차용(메시지 타입이 `PositionTarget`이고 CBF가 붙으므로 MUX 자체는 커스텀). 안전모니터는 두 도구를 가짐: ① 최상위 setpoint를 MUX에 주입(OFFBOARD 유지한 brake/hold), ② mode 변경(AUTO.LAND)으로 firmware에 인계.

### 6.2 단일 CBF geofence 클램프 (MUX 출력 1곳)
MUX가 고른 단일 명령에 대해서만, publish 직전 현재 위치와 명령 속도로 **경계를 넘기 전에** outward 속도 성분을 제한한다:
- 박스 경계마다 `h(x) ≥ 0`(내부), 제어배리어 `ḣ(x,v) ≥ -α·h(x)` 강제 → 경계 접근 시 허용 outward 속도 → 0(제동거리 내 정지).
- inward/접선 속도는 통과 → 복귀 명령을 막지 않음. **어느 명령원이든(emergency 포함) 박스 이탈은 동일하게 불허** → 클램프가 명령원마다 흩어지지 않고 한 곳.

### 6.3 mode guard
`/mavros/state`를 구독해 **OFFBOARD일 때만 발행**한다. 파일럿이 RC로 takeover(§6.4)하면 PX4가 OFFBOARD를 이탈 → MUX 출력은 PX4에 무시되어 **자동 무력화**, 재진입 전까지 idle 유지.

### 6.4 파일럿 takeover는 firmware가 담당 (MUX 위에 있음)
"언제 어디서나 RC를 움직이면 즉시 우선"은 **companion MUX가 아니라 `COM_RC_OVERRIDE`(firmware)** 가 맞다:
- RC는 물리적으로 FCU 수신기로 들어간다. companion이 `/mavros/rc/in`으로 중계·우선화하면 **왕복 지연 + companion 의존**이고, PX4는 여전히 OFFBOARD라 *진짜 인계가 아님*.
- `COM_RC_OVERRIDE`는 스틱이 임계(`COM_RC_STICK_OV`) 초과 시 PX4가 **즉시 OFFBOARD 이탈 → 수동 모드** — companion 무관·무지연·진짜 인계.
- 따라서 **최상위 우선순위(파일럿)는 MUX 밖 firmware**에 있고, MUX는 그 아래 소프트웨어 명령원만 중재한다. A1(companion 불사)을 믿더라도 이건 firmware로 두는 게 옳다.

### 6.5 핵심 결합/한계
- CBF의 위치원은 **EKF2 local_position = vision 의존** → vision이 망가지면 geofence도 함께 무력. 신뢰를 추정 health에 게이트(F4/F5 시 박스 축소 또는 즉시 하강).

---

## 7. A′: 학습형 관성 velocity fallback (AirIO)

근거 논문: *AirIO: Learning Inertial Odometry with Enhanced IMU Feature Observability* (RA-L 2025).

### 7.1 왜 작동하나
IMU를 body frame 유지 + attitude 명시 인코딩 → 중력이 자세와 선형 결합되어 **관성 feature 관측성 회복**. 적분이 아니라 `(attitude, IMU) → body velocity` 회귀 + per-axis uncertainty. 외부센서·thrust 입력 불요.

### 7.2 우리 셋업에서의 적용
- **학습 데이터 = mocap.** IMU + mocap-GT body velocity로 본 드론·본 IMU 학습(A4). 비상시 AirIO가 대체할 그 mocap이 곧 학습 도구.
- **반드시 hover·저속 하강을 학습 분포에 포함** — 비상 하강이 그 영역이며, 논문 학습분포(aggressive racing)의 OOD.
- **통합**: AirIO body velocity → `/mavros/vision_speed/speed_body` → EKF2 velocity aiding. vz도 나오므로 baro 하강률을 보완.
- **guard**: AirIO cov를 L1이 감시(F5). 신뢰 철회 시 F6로.

### 7.3 검증 단계
1. **오프라인 probe(무비행)**: 기존/신규 bag으로 mocap velocity(=GT) vs AirIO 예측 velocity의 RTE@5s 측정. 본 드론에서 sub-meter 재현 확인.
2. **open-loop 통합**: 비행 중 AirIO를 *수동적으로* 돌려 mocap과 실시간 일치도 로깅(fallback 동작은 아직 안 함).
3. **closed-loop**: mocap을 의도적으로 끊고 AirIO fallback 하강을 테더/저고도에서 시험.

### 7.4 compute 위험
Orin AGX inference ≈ **74ms(~13Hz)**. PyTorch CUDA가 **JAX와 통합 RAM 경합**(JAX prealloc 0.25 캡한 그 이슈) → **OOM 위험**. RAM 예산을 명시 측정 후 통합. (A1이 깨지는 주요 경로 중 하나 — §10.)

---

## 8. 미해결 VERIFY-BEFORE-FLIGHT 항목

| V# | 항목 | 위험 |
|---|---|---|
| V1 | `_send_to_mavros`가 JAX **body Z-UP velocity를 FRAME_LOCAL_NED로 부호/프레임 변환 없이** 발행 | 명령 방향 반전/축 뒤바뀜 → 즉시 사고 |
| V2 | vision_pose 프레임 정렬(Motive 축 ↔ EKF2 ENU) 미검증 | 융합된 local_position이 틀어짐 |
| V3 | `/mavros/imu/data`는 MAVROS 기동 후에만 존재 | AirIO/JAX 기동 순서 의존 |
| V4 | EKF2 innovation gate / RC `GF_*` 등 PX4 안전 파라미터가 **레포에 0건** | FCU 거동이 미관리·미지 |
| V5 | AirIO open/closed-loop 미검증(§7.3) | fallback이 실제로 통제 하강을 주는지 미확인 |

---

## 9. 마일스톤

구현은 별도 `flight_safety` repo에서 YAML SoT 기반으로 진행(§11). 각 M은 게이트.

- **M0 — firmware SoT(무비행)**: FCU 파라미터 레포 덤프 + firmware 3종(innovation gate, `GF_*`, `COM_RC_OVERRIDE`) 설정. V1~V3 벤치 검증.
- **M1 — Fault 모니터링 (observe-only)** ★첫 마일스톤: 모든 소스 fault 규명(§11 카탈로그) + 모니터 전부 enable, **개입 없음**. 평시 비행에 수동 동반해 모니터가 실제 fault에 발화하는지 검증.
- **M1.5 — AirIO 오프라인 probe(R&D, 병렬)**: §7.3-1. 본 드론 RTE@5s sub-meter 확인. **통과해야 통합 진행.**
- **M2 — Estimation MUX**: §3.1. vision 소스 단일화(현 2-publisher 충돌 해소) + supervisor health 게이트.
- **M3 — Command MUX + 단일 CBF**: §6. 소프트웨어 명령원 우선순위 중재 + MUX 출력 1곳 클램프 + mode guard.
- **M4 — 개입 권한 + AirIO 통합**: §5 fault 매트릭스를 action으로 연결. AirIO → vision_speed. AUTO.LAND 경로.
- **M5 — 통합 비행시험**: 테더 → 저공 → 엔벨로프 확장. fault injection. V5 종료.

---

## 10. 잔존 위험 등록부

| R# | 위험 | 원인 가정 | 완화(현재) | 미완화분 |
|---|---|---|---|---|
| R1 | **Companion 사망 = 무방비 추락** | A1 | 없음(1차에서 L0 제외) | firmware AUTO.LAND/`GF_*`/`COM_RC_OVERRIDE` 미설정 시 **A1이 깨지면 backstop 0.** OOM(R3)·열·전원·커널패닉이 트리거. → 권고: 비용 거의 0인 firmware 3종(§4 각주)만이라도 설정 — 특히 `COM_RC_OVERRIDE`는 companion이 죽어도 파일럿 인계를 보장 |
| R2 | VRPN+AirIO 동시 상실(F6) 시 수평 blind | A3, A4 | 자세유지+baro 하강 + 저고도 disarm + 엔벨로프 저고도·소박스 | 좁은 실내서 수평 표류 → 충돌 가능. 생존성은 엔벨로프 크기에 비례 |
| R3 | AirIO/JAX 통합 RAM OOM | §7.4 | RAM 예산 측정(M1.5) | 미측정. R1의 주 트리거이기도 |
| R4 | geofence가 vision과 동반 실패 | §6.3 | health 게이트(F4/F5 시 박스 축소) | vision 급변 순간의 짧은 창은 노출 |
| R5 | AirIO가 OOD(저속 하강)서 부정확 | A4 | 학습분포에 hover/하강 포함 | 학습 커버리지에 의존 |

---

## 11. 구현: 별도 repo + YAML SoT

안전 스택은 **`flight_safety` 별도 git repo**로 관리하고(catkin ws에 vendoring), 모든 소스/임계값/우선순위/박스를 **YAML SoT**로 둔다. 핵심 불변식: **1차 vision 소스(mocap ↔ fast-livo) 전환은 `estimation_mux.yaml`의 우선순위 리스트 순서 변경만으로 가능해야 하며, 코드 변경이 없어야 한다.** (기존 `/system/localization` 직교축 패턴과 동일 철학.)

파일단위 아키텍처, fault 카탈로그(by source), config 스키마, monitor→supervisor→mux 계약은 해당 repo의 `docs/`로 이관한다. 본 문서(설계)는 drone-stack-docker에 남기거나 함께 이관.
