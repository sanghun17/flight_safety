from types import SimpleNamespace

import pytest
import rospy

from flight_safety.response import Response


def response(monkeypatch):
    monkeypatch.setattr(rospy.Time, 'now', staticmethod(lambda: rospy.Time.from_sec(10.)))
    r=Response.__new__(Response)
    r.allow_external_termination=True;r.fcu_received=rospy.Time.from_sec(9.9)
    r.ground_received=rospy.Time.from_sec(9.9);r.on_ground=True
    r.fcu_connected=True;r.armed=False;r.mode='POSCTL';r.fault=SimpleNamespace(level=0)
    r.termination_busy=False;r.killed=True;r.external_termination_latched=True
    r._kill=lambda:False;r._reset_control_session=lambda reason:None
    return r


def test_completed_mission_can_prepare_next_pilot_session(monkeypatch):
    r=response(monkeypatch)
    assert r._reset_external_termination(None).success
    assert not r.killed and not r.external_termination_latched


@pytest.mark.parametrize('field,value', [('armed',True),('mode','OFFBOARD'),
    ('mode','AUTO.LAND'),('on_ground',False),('fcu_connected',False),
    ('allow_external_termination',False),('termination_busy',True),
    ('external_termination_latched',False),('fcu_received',rospy.Time.from_sec(1.)),
    ('ground_received',rospy.Time.from_sec(1.))])
def test_reset_does_not_release_other_termination_states(monkeypatch,field,value):
    r=response(monkeypatch);setattr(r,field,value)
    assert not r._reset_external_termination(None).success
    assert r.killed


def test_fault_and_pilot_kill_remain_authoritative(monkeypatch):
    r=response(monkeypatch);r.fault.level=2
    assert not r._reset_external_termination(None).success
    r.fault.level=0;r._kill=lambda:True
    assert not r._reset_external_termination(None).success


@pytest.mark.parametrize('admitted,fresh,preserve', [(True,True,True),(False,True,False),(True,False,False)])
def test_arm_edge_preserves_only_current_admitted_prearm_stream(monkeypatch,admitted,fresh,preserve):
    r=response(monkeypatch);r._state_initialized=True;r.mode='OFFBOARD'
    r.normal=object();command=r.normal;r.normal_stamp=rospy.Time.from_sec(9.9)
    r._offboard_admitted=admitted;r._normal_is_fresh=lambda now:fresh
    r._reset_control_session=lambda reason:(setattr(r,'normal',None),setattr(r,'_offboard_admitted',False))
    r._on_state(SimpleNamespace(connected=True,armed=True,mode='OFFBOARD'))
    assert (r.normal is command)==preserve
    assert r._offboard_admitted==preserve


def test_admitted_arm_edge_never_temporarily_clears_normal(monkeypatch):
    r=response(monkeypatch);r._state_initialized=True;r.mode='OFFBOARD'
    r.normal=object();command=r.normal;r.normal_stamp=rospy.Time.from_sec(9.9)
    r._offboard_admitted=True;r._normal_is_fresh=lambda now:True
    r.idle_hold=object()
    def reject_transient_reset(reason):
        pytest.fail('Concurrent response timer could observe an empty Normal stream')
    r._reset_control_session=reject_transient_reset
    r._on_state(SimpleNamespace(connected=True,armed=True,mode='OFFBOARD'))
    assert r.normal is command and r._offboard_admitted
    assert r.idle_hold is None
