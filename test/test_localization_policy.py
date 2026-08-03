from diagnostic_msgs.msg import DiagnosticStatus

from flight_safety.diagnosis import consistency, geofence
from flight_safety.monitor import Monitor


class FakeDuration(object):
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def to_sec(self):
        return self.seconds


class FakeTime(object):
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def __sub__(self, other):
        return FakeDuration(self.seconds - other.seconds)


class Clock(object):
    current = 0.0

    @classmethod
    def now(cls):
        return FakeTime(cls.current)


class FakeStat(object):
    def __init__(self):
        self.level = None
        self.message = None
        self.values = {}

    def add(self, key, value):
        self.values[key] = value

    def summary(self, level, message):
        self.level = level
        self.message = message


def make_geofence(monkeypatch, primary_age, fallback_age, position=(0.0, 0.0, 1.0)):
    monkeypatch.setattr(geofence.rospy, "Time", Clock)
    diag = geofence.GeofenceDiag.__new__(geofence.GeofenceDiag)
    diag.box = {"x": [-2.5, 2.5], "y": [-2.5, 2.5], "z": [-0.5, 2.0]}
    diag.margin = 0.4
    diag.timeout = 0.5
    diag.fallback_timeout = 0.5
    diag.pos = position
    diag.last_rx = None if primary_age is None else FakeTime(Clock.current - primary_age)
    diag.last_fallback_rx = (
        None if fallback_age is None else FakeTime(Clock.current - fallback_age))
    return diag


def run_diag(diag):
    stat = FakeStat()
    diag.run_diag(stat)
    return stat


def test_geofence_vrpn_gap_with_healthy_local_position_requests_land(monkeypatch):
    Clock.current = 10.0
    stat = run_diag(make_geofence(monkeypatch, primary_age=0.6, fallback_age=0.01))
    assert stat.level == DiagnosticStatus.WARN
    assert "controlled LAND" in stat.message


def test_geofence_both_position_sources_missing_requests_kill(monkeypatch):
    Clock.current = 10.0
    stat = run_diag(make_geofence(monkeypatch, primary_age=0.6, fallback_age=0.6))
    assert stat.level == DiagnosticStatus.ERROR
    assert "KILL" in stat.message


def test_geofence_outside_remains_kill_even_with_healthy_fallback(monkeypatch):
    Clock.current = 10.0
    diag = make_geofence(
        monkeypatch, primary_age=0.01, fallback_age=0.01,
        position=(2.6, 0.0, 1.0))
    stat = run_diag(diag)
    assert stat.level == DiagnosticStatus.ERROR
    assert "OUTSIDE" in stat.message


def test_geofence_379ms_gap_is_inside_new_500ms_timeout(monkeypatch):
    Clock.current = 10.0
    stat = run_diag(make_geofence(monkeypatch, primary_age=0.37927, fallback_age=0.01))
    assert stat.level == DiagnosticStatus.OK
    assert "INSIDE" in stat.message


def make_consistency(monkeypatch, pair_age, error):
    monkeypatch.setattr(consistency.rospy, "Time", Clock)
    diag = consistency.ConsistencyDiag.__new__(consistency.ConsistencyDiag)
    diag.pair_timeout = 0.5
    diag.warn = 0.10
    diag.error = 0.25
    diag.err = error
    diag.last_pair = None if pair_age is None else FakeTime(Clock.current - pair_age)
    return diag


def test_consistency_missing_or_stale_pair_requests_land(monkeypatch):
    Clock.current = 10.0
    for pair_age in (None, 0.6):
        stat = run_diag(make_consistency(monkeypatch, pair_age, error=0.0))
        assert stat.level == DiagnosticStatus.WARN


def test_consistency_fresh_large_mismatch_remains_kill(monkeypatch):
    Clock.current = 10.0
    stat = run_diag(make_consistency(monkeypatch, pair_age=0.01, error=0.30))
    assert stat.level == DiagnosticStatus.ERROR
    assert "MISMATCH" in stat.message


def test_monitor_caps_vrpn_error_to_land_but_not_local_position_error():
    now = FakeTime(10.0)
    monitor = Monitor.__new__(Monitor)
    monitor.sources = [
        ("pure stream", 3.0, DiagnosticStatus.WARN),
        ("local_position", 3.0, DiagnosticStatus.ERROR),
    ]
    monitor.last = {
        "pure stream": (DiagnosticStatus.ERROR, "no data", FakeTime(9.9)),
        "local_position": (DiagnosticStatus.OK, "alive", FakeTime(9.9)),
    }

    level, names, messages = monitor.worst(now)
    assert level == DiagnosticStatus.WARN
    assert names == ["pure stream"]
    assert "policy cap: 2->1" in messages[0]

    monitor.last["local_position"] = (
        DiagnosticStatus.ERROR, "DEAD", FakeTime(9.9))
    level, names, _ = monitor.worst(now)
    assert level == DiagnosticStatus.ERROR
    assert names == ["local_position"]
