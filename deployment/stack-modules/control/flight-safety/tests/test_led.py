import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch
import rospy

spec=importlib.util.spec_from_file_location('status_led',Path(__file__).resolve().parents[1]/'fs_led_node.py')
led=importlib.util.module_from_spec(spec)
gpio=SimpleNamespace()
with patch.dict(sys.modules,{'Jetson':SimpleNamespace(GPIO=gpio),'Jetson.GPIO':gpio}):spec.loader.exec_module(led)


class LEDTest(unittest.TestCase):
    def setUp(self):
        self.time=patch.object(led.rospy.Time,'now',return_value=rospy.Time.from_sec(100));self.time.start()
        self.addCleanup(self.time.stop)
        self.n=led.LedNode.__new__(led.LedNode)
        self.n.state=SimpleNamespace(control_lane='NORMAL',mode='OFFBOARD',level=0,armed=False)
        self.n.state_last=self.n.rc_last=self.n.sp_last=rospy.Time.from_sec(99.9)
        self.n.kill_high=False;self.n.mode_offb=True;self.n.mission=None;self.n.mission_last=None
    def hint(self,state='hold_failed',dry=False):self.n._mission_cb(SimpleNamespace(data=json.dumps(dict(state=state,dry_run=dry))))
    def test_default_unchanged(self):self.assertEqual(self.n._decide(),(led.GREEN,led.OFF,0.))
    def test_failed_hover(self):self.hint();self.assertEqual(self.n._decide(),(led.GREEN,led.WHITE,2.))
    def test_kill_and_emergency_override_mission(self):
        self.hint();self.n.kill_high=True;self.assertEqual(self.n._decide(),(led.RED,led.GREEN,4.))
        self.n.kill_high=False;self.n.state.control_lane='KILL';self.assertEqual(self.n._decide(),(led.RED,led.RED,0.))
        self.n.state.control_lane='LAND';self.n.state.level=1;self.assertEqual(self.n._decide(),(led.AMBER,led.GREEN,2.))
    def test_dry_stale_and_bad_input_ignored(self):
        self.hint(dry=True);self.assertEqual(self.n._decide(),(led.GREEN,led.OFF,0.))
        self.hint();self.n.mission_last=rospy.Time.from_sec(98);self.assertEqual(self.n._decide(),(led.GREEN,led.OFF,0.))
        self.n._mission_cb(SimpleNamespace(data='not json'));self.assertIsNone(self.n.mission)
    def test_starvation_not_hidden(self):
        self.hint();self.n.sp_last=rospy.Time.from_sec(98);self.assertEqual(self.n._decide(),(led.GREEN,led.OFF,2.))
    def test_intentional_land_distinct_from_offboard_loss(self):
        self.n.state.mode='AUTO.LAND';self.n.state.control_lane='MANUAL'
        self.assertEqual(self.n._decide(),(led.GREEN,led.BLUE,2.))
        self.hint('landing');self.assertEqual(self.n._decide(),(led.CYAN,led.OFF,2.))
    def test_manual_return_not_failed_hover_color(self):
        self.hint();self.n.state.mode='POSCTL';self.n.state.control_lane='MANUAL';self.n.mode_offb=False
        self.assertEqual(self.n._decide(),(led.BLUE,led.OFF,0.))

    def ground_hint(self,ready=True):
        self.n.state.mode='POSCTL';self.n.state.control_lane='MANUAL';self.n.mode_offb=False
        self.n._mission_cb(SimpleNamespace(data=json.dumps(dict(state='idle',dry_run=False,
            ground_start_enabled=True,offboard_entry_ready=ready))))

    def test_ground_ready_and_wait_distinct(self):
        self.ground_hint();self.assertEqual(self.n._decide(),(led.CYAN,led.OFF,0.))
        self.ground_hint(False);self.assertEqual(self.n._decide(),(led.BLUE,led.WHITE,2.))

    def test_ground_hint_never_masks_fault_kill_or_offboard_loss(self):
        self.ground_hint();self.n.kill_high=True
        self.assertEqual(self.n._decide(),(led.RED,led.BLUE,4.))
        self.n.kill_high=False;self.n.state.control_lane='KILL'
        self.assertEqual(self.n._decide(),(led.RED,led.RED,0.))
        self.n.state.control_lane='MANUAL';self.n.mode_offb=True
        self.assertEqual(self.n._decide(),(led.GREEN,led.BLUE,2.))
        self.n.mode_offb=False;self.n.state.level=1
        self.assertEqual(self.n._decide(),(led.BLUE,led.OFF,0.))

    def test_ready_requires_disarmed_fresh_live_hint(self):
        self.ground_hint();self.n.state.armed=True
        self.assertEqual(self.n._decide(),(led.BLUE,led.OFF,0.))
        self.n.state.armed=False;self.n.mission_last=rospy.Time.from_sec(98)
        self.assertEqual(self.n._decide(),(led.BLUE,led.OFF,0.))
        self.ground_hint();self.n._mission_cb(SimpleNamespace(data='{"dry_run":true}'))
        self.assertEqual(self.n._decide(),(led.BLUE,led.OFF,0.))

if __name__=='__main__':unittest.main()
