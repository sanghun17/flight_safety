# Standalone VIO: fail-closed FCU audit and subscriber-only shadow evidence

이 도구들은 실비행 설정을 적용하지 않는다. FCU 상태/parameter를 읽어 증거화하고 VIO 및
hybrid-IMU 출력을 구독 계측할 뿐이다. 기존 `safety.launch`, `run.sh`, estimator mux 기본값은
변경하지 않으며, 어느 receipt의 `PASS`도 arm, offboard, EKF fusion 또는 비행을 허가하지 않는다.

이번 구현·테스트에서는 실제 ROS master, MAVROS, FCU를 호출하거나 package를 build/deploy하지
않았다. 실제 수집은 별도 승인 후 propeller를 제거하고 disarmed/on-ground 상태에서만 수행한다.

## 1. FCU read-only audit v2

### 실행 순서와 안전 guard

collector는 다음 순서를 고정한다.

1. 실제 policy YAML을 exact schema로 검증하고 bundle의 `fcu_audit_config` role/path/SHA에 결속한 뒤,
   collector/MAVROS source, package version, runtime topic type 및 message MD5 identity를 읽는다.
   `/use_sim_time=false`도 pull 전과 pull 직전에 확인한다.
2. vehicle info의 PX4 autopilot(12), sysid, compid, UID, firmware/custom version을 엄격한 타입으로
   확인한다. Python의 `bool`을 integer 증거로 허용하지 않는다.
3. state/extended/param/timesync/estimator topic publisher와 vehicle-info/param-pull service provider가
   정확히 하나의 local MAVROS node PID에 속하고 그 PID가 exact param-plugin binary를 mapping하는지
   확인한다. accepted-sequence publisher도 local PID와 실제 `/proc/<pid>/exe`에 결속한다. 그 뒤
   source/package/message/bundle identity와 `/mavros/state`, `/mavros/extended_state`의 두 개 이상 fresh
   precheck가 모두 통과하고 connected + explicitly disarmed +
   `LANDED_STATE_ON_GROUND`일 때만 parameter pull을 시작한다.
4. collector-owned subscriber가 `/mavros/param/param_value`, state, extended_state 각각의 publisher
   connection을 확인하고 leading ready record를 실제 arrival ROS/steady clock과 함께 기록한 뒤에만
   state/extended_state에서 첫 actual-arrival sample이 connected/disarmed/on-ground임을 다시 확인한
   뒤에만 exact-bound `mavparam` entrypoint의 `dump -f`를 실행한다. process가 단지 살아 있거나
   publisher connection만 존재하는 것은 safety-sample handshake가 아니다.
5. pull 중 state/extended_state의 **steady-clock actual arrival**가 dump 시작 이전부터 종료 이후까지
   span하고, 각 header도 해당 actual ROS arrival 시점에 fresh이며, 최소 sample/cadence 및
   safe/monotonic/continuous 조건을 만족하는지 확인한 뒤,
   pull 직후 fresh postcheck를 다시 수행한다.

precheck 또는 package/message/source/bundle/runtime graph identity가 unsafe/missing이면 force pull 자체를 실행하지
않는다. 수집 중 또는 postcheck에서 arm,
disconnect, in-air, stale/gap을 한 번이라도 보면 receipt는 fail-closed `FAIL`이다. 증거/identity가
없거나 재구성이 불가능하면 `INCOMPLETE`이고 `PASS`가 아니다.

### full parameter completeness

CSV 행 개수나 `mavparam` stdout만 신뢰하지 않는다. 동시에 수집한
`mavros_msgs/Param` raw trace에서 다음을 전부 확인한다.

- 하나의 advertised real `param_count=N`
- real index 집합이 정확히 `0..N-1`
- name/index/value/count의 일관성과 name/index uniqueness
- exact retry duplicate만 허용하고, 충돌하는 duplicate는 실패
- dump name set이 raw real name set과 정확히 같음
- 모든 dump numeric value가 raw decoded value와 정확히 같음
- `mavparam` reported count가 raw set과 일치

PX4의 `_HASH_CHECK`는 MAVROS가 raw topic에 publish한 뒤 ROS parameter cache에 넣지 않는
pseudo-parameter이다. 오직 vehicle `autopilot == 12`, exact name `_HASH_CHECK`, raw
`param_index == 65535`가 함께 증명되고 pseudo record 자체의 `param_count`도 advertised real N 또는
N+1인 경우에만 한 개를 real `0..N-1` set 밖에서 허용한다.
다른 autopilot/name/index 또는 indexed `_HASH_CHECK`는 실패한다.

`mavparam dump -f`는 FCU persistent parameter를 **쓰지 않는 pull/read**이다. 그러나 MAVROS
`param/pull(force_pull=true)`가 FCU 값을 받아 **MAVROS의 ROS parameter cache를 갱신**한다.
receipt는 이 bounded side effect를 명시하며 “zero side effect”라고 표현하지 않는다. 이 경로에는
parameter set/push, arm/disarm, mode, MAVLink command, setpoint, ROS publisher가 없다.

### timesync와 estimator status

timesync `PASS`에는 다음이 모두 필요하다.

- config에 동결된 `conn/timesync_rate`, mode, filter gains, `convergence_window`, strict
  `max_rtt_sample`, deviation/reset threshold, `time/publish_sim_time=false`가 live ROS parameter와
  exact match하고 global `/use_sim_time`도 false
- 동일 값이 supplied MAVROS timesync YAML source에도 존재하고 source SHA-256가 identity capture
  이후 변하지 않음 (C++에 숨은 default만으로는 통과 불가)
- header/remote timestamp monotonic, maximum gap, finish freshness
- TimesyncStatus, `/diagnostics`, typed accepted-sequence를 순차 `rostopic echo`가 아니라 하나의
  source-bound multiplex subscriber window에서 동시에 수집하고, 모든 row에 collector sequence와
  actual arrival ROS/steady stamp를 기록. accepted tail은 같은 window의 status tail과 remote timestamp,
  RTT, residual 및 bounded arrival delay로 one-to-one exact join
- 마지막 tail의 각 sample이 `RTT < max_rtt_sample`이고, current observed offset 대 직전 estimated
  offset residual이 source acceptance threshold와 더 엄격한 profile residual bound를 모두 만족
- 수집한 모든 `DiagnosticStatus`와 순서가 보존된 모든 `KeyValue`가 exact schema이고 duplicate key가
  없으며, 이름이 일치하는 **모든** `mavros: Time Sync` entry가 연속 `level=OK`이다.
  `[ERROR, OK]` tail 회복과 `Timesyncs since startup` counter rollback은 실패한다.
- `Timesyncs since startup`은 observation count일 뿐 accepted filter update count가 아니므로 convergence
  증거로 절대 사용하지 않는다. 별도 typed/source-bound status가 stable session/reset, strictly increasing
  observation sequence, 전체 window accepted counter의 rollback 없는 exact transition, contiguous accepted
  tail sequence, `converged=true`, 각 tail sample의 remote stamp/RTT/
  recomputed deviation 및 publisher source/binary/build SHA를 원자적으로 제공해야 한다. 첫 accepted
  sequence도 `convergence_window` 이상이어야 한다.

Stock MAVROS 1.17은 위 accepted-sequence interface를 제공하지 않는다. 따라서 별도로 qualified된
publisher 증거가 없으면 diagnostic count가 아무리 크고 마지막 20개 observation이 건강해도
`INCOMPLETE`이며 PASS가 아니다.

`/mavros/estimator_status`도 한 snapshot이 아니라 연속 window로 수집한다. 모든 field는 exact bool,
header는 fresh/monotonic/continuous여야 하며 config의 required-true, required-false와
vertical-position-any-true gate를 매 sample 만족해야 한다.
정상적으로 수집된 runtime sample이 threshold/flag를 위반한 경우 evidence 부족인 `INCOMPLETE`가
아니라 측정된 violation인 `FAIL`로 분류한다.

### source/package/message identity

receipt는 다음을 해시/결속한다.

- 실제 실행 collector entrypoint, collector-owned trace entrypoint/core/append-only helper와 source bundle
  manifest
- 실제 pull command인 Python3 interpreter + `mavparam` script, 같은 interpreter probe가 실제 import한
  installed `mavros.param.__file__`/SHA와 runtime-tree `mavros/param.py`, param
  plugin source, MAVROS package 전체 source-tree manifest
- 실행 MAVROS process의 `/proc/<pid>/maps`에 실제 mapping된 param-plugin shared library, CMake cache,
  catkin build marker와 이들을 source tree에 exact join한 runtime build manifest
- MAVROS timesync plugin/effective YAML source 및 typed accepted-sequence publisher source/binary/build
- `rosversion`/`rospack` MAVROS 및 mavros_msgs identity
- live topic type과 `mavros_msgs/{Param,TimesyncStatus,EstimatorStatus,State,ExtendedState,
  VehicleInfo}` MD5
- supplied bundle manifest v3의 required role + exact logical path + SHA-256 mapping. 같은 digest가
  다른 path/role에 존재하는 것만으로는 통과하지 않는다.

repository bundle manifest는 `config/fcu_shadow_bundle_manifest.json`이며 manifest 자체는
순환 self-hash를 피하고 append-only receipt가 실행 시 그 파일의 SHA-256를 별도로 기록한다.

source/package/binary/tree/build/policy/ROS graph 중 하나라도 없거나 서로 join되지 않으면 pre-pull identity가 실패하여
force pull을 호출하지 않는다. 첫 safe trace 뒤 pull 직전 모든 supplied/core digest와 full manifested
MAVROS source tree를 다시 계산하여 identity capture 이후의 TOCTOU 변경도 pull을 막는다. 기본 config의
source path와 아직 존재하지 않는 accepted-sequence
publisher identity는 의도적으로 비어 있어 실제 배포 bundle 경로를 채우기 전에는 `INCOMPLETE`이다.

```bash
source <flight-safety-ws>/devel/setup.bash
# config 사본에 실제 absolute source/bundle paths를 먼저 채운다.
AUDIT_ROOT=/absolute/new/path/fcu-audits
rosrun flight_safety fcu_audit_receipt.py \
  --config /absolute/frozen/fcu_audit_read_only.yaml \
  --output-root "$AUDIT_ROOT"
```

각 run은 append-only 새 디렉터리이며 command stdout/stderr, raw trace, dump, source copies와 receipt가
파일별 SHA-256로 결속된다. exit code는 PASS=0, evidence가 생성된 FAIL/INCOMPLETE=2, receipt를
만들 수 없음=3이다.

Battery readiness는 아직 gate가 아니다. 기체별 battery topic, cell count/chemistry, load-sag model,
threshold가 동결되지 않았으므로 FCU audit PASS도 battery/flight readiness를 뜻하지 않는다.

## 2. Subscriber-only VIO shadow v3

`vio_shadow_preflight.launch`는 shadow node 하나만 시작하며 response/mux/recorder/MAVROS output을
include하지 않는다. publisher, service, relay, parameter set 또는 FCU actuation path가 없다.

### stream/payload/covariance contract

각 stream에서 다음을 raw sample 단위로 확인한다.

- normal requested duration 완료 (`ros_shutdown` 조기 종료 및 과도한 overrun은 실패)
- first-sample delay, 전체-window coverage, arrival/header rate, gap, finish freshness
- zero/duplicate/backward header stamp와 monotonic arrival clock
- exact frame/child-frame 및 arrival-vs-header latency
- position/orientation/twist/IMU numeric payload finite, quaternion norm
- 6x6 covariance finite/nonzero, strictly-positive bounded diagonal, symmetry, full PSD eigenvalue gate,
  maximum absolute bound
- hybrid `sensor_msgs/Imu`의 orientation quaternion 및 orientation/angular-velocity/linear-acceleration
  3x3 covariance 각각에 대해 finite, ROS unknown marker(`covariance[0] == -1`)와 all-zero unknown
  거부, symmetry, PSD, strictly-positive configured diagonal/eigenvalue floor 및 absolute bound

profile은 top-level 및 모든 nested mapping의 allowed key를 exact 검사한다. 오탈자/unknown key는
시작 시 거부되어 silently ignored threshold가 생기지 않는다.

모든 raw observation은 non-finite number를 explicit string으로 보존한 canonical JSONL artifact에
기록한다. artifact SHA-256, record count, profile, metric-core source SHA-256가 receipt에 함께 있어
summary를 재계산할 수 있다. summary만 남기고 raw를 버리지 않는다.

### estimator session/correction linkage

별도 String session과 scalar UInt32 reset/status가 동시에 보이는 것만으로는 one-to-one을 증명할 수
없으므로 더 이상 PASS 경로가 없다. 하나의 typed `diagnostic_msgs/DiagnosticArray` status가 exact
schema로 `session_id`, `reset_counter`, `correction_sequence`, `correction_stamp_ns`를 원자적으로
운반해야 한다. 같은 record에는 selected overlay/base/static-camera/build manifest/effective-param/
private-camera-param/executable/source-tree/5개 library SHA-256도 모두 포함되어야 하며 collector가
재계산한 값과 exact match해야 한다. 모든 status level/header/arrival/freshness가 통과하고 session/reset은 window에서
불변, sequence와 correction stamp는 연속/strict-monotonic이어야 한다. 각 correction sample에는
허용 stamp/arrival 범위 안의 typed identity가 정확히 하나만 match하고, propagated header도 가까워야
한다. 현재 estimator에는 이 typed interface가 없으므로 기본 profile은 계속 `FAIL`이다.

identity/hybrid/clock-domain `DiagnosticStatus.values`는 dict로 먼저 축약하지 않고 원래 KeyValue
순서와 중복을 canonical raw evidence에 보존한다. 같은 key가 두 번 나타나면 마지막 값이 그럴듯해도
해당 typed record는 즉시 무효이며 receipt는 `FAIL`이다.

### selected candidate의 hybrid-IMU prerequisite

selected low-rate 후보가 `/camera/imu_hybrid`에 의존하므로 다음을 별도 hard gate로 추가한다.

- `/camera/imu_hybrid` exact output frame, finite gyro/acceleration, >=150 Hz, header/arrival gap,
  duplicate/backward stamp, first delay, coverage, freshness
- latched `/camera/imu_hybrid/ready`가 true가 된 뒤 window 끝까지 false로 내려가지 않음
- `/camera/imu_hybrid/diagnostics`의 `d435i_tools/hybrid_imu`가 연속/fresh/OK/ready이며, 모든
  diagnostic에 profile의 required fault-counter key set이 정확히 존재하고 nonnegative integer
  baseline→final이 monotonic이며 delta=0이고,
  `last_fault`가 바뀌지 않음
- nonempty explicit mapping-start event가 ready 이후 정해진 stable 시간과 최소 OK diagnostic 수를
  채운 뒤 도착하여 같은 session을 결속
- D435 header와 FCU IMU header clock source/domain/schema, identity와 같은 session/reset,
  monotonic measurement sequence, 실제 `d435_stamp_ns`/`fcu_stamp_ns`와 두 값의 차이와 정확히 같은
  signed `measured_offset_ns`, hybrid node/core, D435 launch, MAVROS timesync plugin 및 clock-domain
  publisher executable/source/build-manifest SHA-256를 원자적으로 담은 typed periodic/fresh
  DiagnosticArray status. 별도 hybrid runtime-build manifest는 hybrid node/core/launch digest를 exact
  join하고 clock-domain publisher build manifest는 그 hybrid build와 MAVROS timesync source까지 join한다.
  measurement stamp는 status header와 설정된 freshness 범위 안에 있어야 한다. self-reported SHA만으로는
  부족하며 ROS caller node URI/PID가 collector와 같은 host/PID namespace이고 실제 `/proc/<pid>/exe`
  SHA가 bound publisher executable SHA와 같아야 한다.

현재 hybrid node의 ready는 rate/stale 안정화만 보고 mapping 시작과 결속되지 않는다. 또한 현재
launch에는 mapping-start status 및 D435↔FCU clock-domain status topic이 없다. 기본 profile은 두
topic을 빈 문자열로 두되 둘 다 required=true이며, 현 diagnostics는 zero인 fault key를 모두
advertise하지 않으므로 추정 없이 `FAIL`한다. 이를 해결하기 전에는
hybrid replay score가 좋아도 live VIO flight input으로 qualification할 수 없다. 기존 flight launch를
이 도구가 자동 변경하지 않는다.

### selected Phase-B candidate와 native producer provenance

bundle은 legacy `live_hybrid_imu.yaml`을 selected estimator overlay로 간주하지 않는다. PASS에는
`vio_ground_phase_b_candidate_development.yaml` 역할의 exact path/SHA, FAST-LIVO base config,
base⊕overlay와 extra/missing key 없이 exact type/value로 같은 live ROS parameter 전체 tree,
`camera_d435i.yaml` source 및 `/laserMapping` private namespace에 로드된 static calibration의 전체
exact typed tree, qualified build manifest,
deployed `fastlivo_mapping` executable, 다섯 shared library, manifest에 열거된 모든 estimator source
file digest가 필요하다. 하나라도 누락·불일치하면 producer provenance는 FAIL이다.
두 tree는 canonical JSON SHA-256도 각각 재계산해 expected와 runtime이 같아야 한다. 런타임 전용
ephemeral key는 profile의 두 allowlist에 wildcard 없는 exact `/path/to/key`로 적은 것만 제거할 수
있고, allowlist가 expected config key를 가리키거나 미허용 extra key가 하나라도 있으면 FAIL이다.

selected role은 Phase-B와 같은 `common.online_intrinsics_en: false`를 exact 요구한다. CameraInfo를
사용하는 true profile은 별도의 unqualified shadow profile이며 camera receipt와 rebaseline 없이는 이
candidate와 섞거나 Phase-B 결과를 주장할 수 없다. 기본 runtime namespace/executable/library/source
paths는 비어 있어 계속 FAIL이다.

```bash
source <flight-safety-ws>/devel/setup.bash
# profile 사본에 실제 collector entrypoint/bundle manifest 및 새 explicit
# typed identity/mapping/clock-domain topic과 selected candidate/base/build,
# `/laserMapping` private calibration, deployed executable/library/source/runtime-param 경로를
# 채운 뒤에만 실행한다.
SHADOW_ROOT=/absolute/new/path/vio-shadow
roslaunch flight_safety vio_shadow_preflight.launch \
  config:=/absolute/frozen/vio_shadow_preflight.yaml \
  output_root:="$SHADOW_ROOT" duration_s:=120
```

## 3. 승격 순서

1. FCU firmware/UID/full raw parameter/timesync receipt와 qualified binary manifest를 결속한다.
2. estimator 및 hybrid node에 typed atomic identity join, nonempty mapping-start, exact counter schema,
   source-hashed clock-domain status를
   fail-closed로 구현한다.
3. selected overlay/base/effective params/native build identity를 bind하고, FCU와 연결되지 않은
   subscriber-only bag/live shadow에서 모든 v3 gate를 반복 통과한다.
4. 별도 승인된 propeller-off 시험에서 축/부호, target echo, RC takeover/kill, offboard 및
   position-loss를 검증한다.
5. disarmed EKF innovation/delay shadow, tethered 저속 hover 순으로 별도 승격한다.

현재 zero/invalid covariance high-rate odometry를 `/mavros/odometry/out`에 relay하거나,
`/mavros/vision_pose/pose`, EKF2 parameter, safety mux default를 이 tooling에서 변경하는 것은 범위 밖이다.
