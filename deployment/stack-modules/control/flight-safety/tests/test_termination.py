import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import rospy
from flight_safety.response import Response

class TerminationTest(unittest.TestCase):
    def node(self):
        n=Response.__new__(Response);n.allow_external_termination=True;n.fcu_received=rospy.Time.from_sec(99.9)
        n.fcu_connected=True;n.armed=True;n.mode='OFFBOARD';n._offboard_admitted=True;n.killed=False
        n._manual=lambda:False;n._normal_is_fresh=lambda _:True;n._kill=lambda:False
        n.actions=SimpleNamespace(kill=Mock(return_value=True));return n
    @patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.))
    def test_admitted_request_once(self,_):
        n=self.node();self.assertTrue(n._request_termination(None).success);self.assertTrue(n.killed)
        self.assertFalse(n._request_termination(None).success);n.actions.kill.assert_called_once()
    @patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.))
    def test_rejected_gates_never_call_actuator(self,_):
        for k,v in [('allow_external_termination',False),('armed',False),('mode','POSCTL'),('fcu_connected',False),('fcu_received',rospy.Time.from_sec(97.)),('_offboard_admitted',False),('_manual',lambda:True),('_normal_is_fresh',lambda _:False)]:
            n=self.node();setattr(n,k,v);self.assertFalse(n._request_termination(None).success);n.actions.kill.assert_not_called()
    @patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.))
    def test_rejected_fcu_ack_not_latched(self,_):
        n=self.node();n.actions.kill.return_value=False
        self.assertFalse(n._request_termination(None).success);self.assertFalse(n.killed)
