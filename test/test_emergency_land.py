"""Command arbitration/termination tests without a ROS master or real services."""
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
import rospy
from mavros_msgs.msg import State,ExtendedState,PositionTarget
from flight_safety.msg import Fault
from flight_safety.response import Response,WARN,ERROR


@pytest.fixture
def node(monkeypatch):
    clock=[10.]
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time.from_sec(clock[0])))
    monkeypatch.setattr(rospy,'get_param',lambda key,default=None:default)
    monkeypatch.setattr(rospy,'Publisher',lambda *a,**kw:Mock())
    monkeypatch.setattr(rospy,'Subscriber',Mock());monkeypatch.setattr(rospy,'Service',Mock())
    monkeypatch.setattr(rospy,'Timer',Mock())
    monkeypatch.setattr('flight_safety.response.MavrosActions',Mock)
    r=Response();r.armed=True;r.mode='OFFBOARD';r.fcu_connected=True
    r.fcu_received=rospy.Time.now();r._offboard_admitted=True;r._state_initialized=True
    r.actions.kill.return_value=True
    return r,clock


def normal(r,z=1.59):
    m=PositionTarget();m.coordinate_frame=m.FRAME_LOCAL_NED;m.position.z=z
    r._on_normal(m)


def tick(r,clock,now):
    clock[0]=now;r.fcu_received=rospy.Time.now();r._tick(None)


def ground(r):
    m=ExtendedState();m.header.stamp=rospy.Time.now();m.landed_state=m.LANDED_STATE_ON_GROUND;r._on_extended(m)


def test_cleared_warn_cannot_restore_fresh_normal_or_idlehold(node):
    r,c=node;normal(r);r.fault=Fault(level=WARN);r._tick(None)
    assert r.land_latched and r.in_land and r.normal is None and r.idle_hold is None
    r.fault=Fault()
    for now in [10.1,10.3,10.8]:
        c[0]=now;normal(r);tick(r,c,now)
        output=r.out.publish.call_args.args[0]
        assert output.velocity.z==-r.descend and output.type_mask&PositionTarget.IGNORE_PZ
        assert r.normal is None and r.state_pub.publish.call_args.args[0].control_lane=='LAND'
    r.actions.kill.assert_not_called()


def test_post_entry_continuous_ground_disarms_without_imu_impact(node):
    r,c=node;r.on_ground=True;r.ground_received=rospy.Time.from_sec(9.9)
    r.fault=Fault(level=WARN);r._tick(None);r.fault=Fault()
    tick(r,c,10.4);r.actions.kill.assert_not_called() # old ground report never qualifies
    ground(r);tick(r,c,10.6);r.actions.kill.assert_not_called()
    c[0]=10.75;ground(r);tick(r,c,10.75)
    r.actions.kill.assert_called_once_with();assert r.killed
    before=r.out.publish.call_count;normal(r);tick(r,c,10.8)
    assert r.out.publish.call_count==before


@pytest.mark.parametrize('kind',['ground','fcu','disconnected'])
def test_stale_or_disconnected_state_never_confirms_ground(node,kind):
    r,c=node;r.fault=Fault(level=WARN);r._tick(None);c[0]=10.1;ground(r)
    c[0]=11.2;r.fcu_received=rospy.Time.now()
    if kind!='ground':r.ground_received=rospy.Time.now()
    if kind=='fcu':r.fcu_received=rospy.Time.from_sec(8.)
    if kind=='disconnected':r.fcu_connected=False
    r._tick(None);r.actions.kill.assert_not_called()


def test_airborne_report_resets_ground_confirmation(node):
    r,c=node;r.fault=Fault(level=WARN);r._tick(None)
    c[0]=10.1;ground(r)
    m=ExtendedState();m.landed_state=m.LANDED_STATE_IN_AIR;r._on_extended(m)
    tick(r,c,10.6);r.actions.kill.assert_not_called()


def test_pilot_takeover_suspends_land_and_disarm(node):
    r,c=node;r.fault=Fault(level=WARN);r._tick(None);r.fault=Fault()
    r._on_state(State(connected=True,armed=True,mode='POSCTL'))
    before=r.out.publish.call_count;c[0]=10.1;ground(r);tick(r,c,10.6)
    assert r.out.publish.call_count==before;r.actions.kill.assert_not_called()
    assert r.land_latched and r.state_pub.publish.call_args.args[0].control_lane=='MANUAL'
    r._on_state(State(connected=True,armed=False,mode='POSCTL'))
    assert not r.land_latched


def test_disarm_retry_is_bounded_and_stops_on_confirmation(node):
    r,c=node;r.actions.kill.side_effect=[False,True]
    r.fault=Fault(level=WARN);r._tick(None);c[0]=10.1;ground(r);tick(r,c,10.5)
    assert r.killed and r.actions.kill.call_count==1
    tick(r,c,11.);assert r.actions.kill.call_count==1
    tick(r,c,11.6);assert r.actions.kill.call_count==2
    r._on_state(State(connected=True,armed=False,mode='OFFBOARD'))
    tick(r,c,12.8);assert r.actions.kill.call_count==2


def test_imu_touchdown_and_error_escalation_still_take_priority(node):
    r,c=node;r.fault=Fault(level=WARN);r._tick(None);r.fault=Fault()
    r.touchdown_fired=True;tick(r,c,10.1);assert r.killed;r.actions.kill.assert_called_once()


def test_error_overrides_latched_land(node):
    r,c=node;r.fault=Fault(level=WARN);r._tick(None);r.fault=Fault(level=ERROR)
    tick(r,c,10.1);assert r.killed
    assert r.state_pub.publish.call_args.args[0].control_lane=='KILL'


def test_existing_error_authority_does_not_require_fresh_fcu_for_first_kill(node):
    r,c=node;r.fcu_received=rospy.Time.from_sec(1.);r.fault=Fault(level=ERROR)
    r._tick(None);r.actions.kill.assert_called_once()


def test_stale_measurement_header_cannot_confirm_ground(node):
    r,c=node;r.fault=Fault(level=WARN);r._tick(None)
    m=ExtendedState();m.header.stamp=rospy.Time.from_sec(1.);m.landed_state=m.LANDED_STATE_ON_GROUND
    c[0]=10.1;r._on_extended(m);tick(r,c,10.5)
    assert not r.on_ground;r.actions.kill.assert_not_called()


def test_ground_session_reset_during_tick_never_kills_state_timer(node,monkeypatch):
    r,c=node;normal(r)
    publish_state=r._publish_state
    def reset_between_freshness_check_and_forward(*args):
        publish_state(*args)
        r._reset_control_session('concurrent ground-mode callback')
    monkeypatch.setattr(r,'_publish_state',reset_between_freshness_check_and_forward)
    r._tick(None)
    r.out.publish.assert_not_called()
    assert r.state_pub.publish.call_count==1
    r._tick(None)
    assert r.state_pub.publish.call_count==2
