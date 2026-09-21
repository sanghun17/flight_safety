from pathlib import Path
from unittest.mock import patch
import unittest
import rospy
import yaml
from flight_safety.diagnosis.geofence import GeofenceDiag,INSIDE,APPROACHING,OUTSIDE,UNKNOWN

ROOT=Path(__file__).resolve().parents[4]
class ProfilesTest(unittest.TestCase):
    def node(self,stack):
        cfg=yaml.safe_load((ROOT/'stacks'/stack/'config/geofence.yaml').read_text())
        with patch('rospy.Subscriber'),patch('rospy.Publisher'),patch('rospy.Timer'):
            n=GeofenceDiag(cfg)
        n.pos=(0.,0.,0.);n.last_rx=rospy.Time.from_sec(10.)
        return n
    def status(self,n,p):n.pos=p;return n.status(rospy.Time.from_sec(10.))[0]
    def test_aruco_unbounded_z_but_xy_preserved(self):
        n=self.node('aruco-landing-jetson')
        for z in [-100.,-.6,0.,1.7,2.,100.]:self.assertEqual(self.status(n,(0.,0.,z)),INSIDE)
        self.assertEqual(self.status(n,(2.3,0.,100.)),APPROACHING)
        self.assertEqual(self.status(n,(0.,-2.6,100.)),OUTSIDE)
    def test_risk_existing_ceiling_and_floor_preserved(self):
        n=self.node('d435i-voxblox')
        self.assertEqual(self.status(n,(0.,0.,1.)),INSIDE)
        self.assertEqual(self.status(n,(0.,0.,1.7)),APPROACHING)
        self.assertEqual(self.status(n,(0.,0.,2.1)),OUTSIDE)
        self.assertEqual(self.status(n,(0.,0.,-.6)),OUTSIDE)
    def test_xy_marker_has_no_ceiling_and_stale_pose_still_unknown(self):
        n=self.node('aruco-landing-jetson');n.pos=(0.,0.,3.)
        edges=n._fence_edges();self.assertEqual(len(edges),4)
        self.assertTrue(all(p[2]==3 for edge in edges for p in edge))
        self.assertEqual(n.status(rospy.Time.from_sec(11.))[0],UNKNOWN)
        self.assertEqual(len(self.node('d435i-voxblox')._fence_edges()),12)
    def test_risk_profile_matches_legacy_boundary_settings(self):
        new=yaml.safe_load((ROOT/'stacks/d435i-voxblox/config/geofence.yaml').read_text())
        old=yaml.safe_load((ROOT/'ws/flight-safety/src/flight_safety/config/diagnosis.yaml').read_text())['geofence']
        self.assertEqual(new.pop('enabled_axes'),['x','y','z']);self.assertEqual(new,old)
