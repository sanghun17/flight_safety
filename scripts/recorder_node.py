#!/usr/bin/env python
"""Arm-triggered rosbag recorder.

Watches /mavros/state: on ARM (disarmed->armed) it spawns `rosbag record` of the
configured topics into ~bag_dir/<prefix>_<stamp>.bag; on DISARM it stops the
recorder with SIGINT so rosbag finalizes the index cleanly (NOT SIGKILL -> that
leaves an unindexed bag needing `rosbag reindex`). One bag per arming.

Params (set in launch/config/recorder.yaml):
  ~bag_dir     output dir (default /work/flight_logs)
  ~prefix      bag basename prefix (default "flight")
  ~record_all  true -> `rosbag record -a` (everything); else record ~topics
  ~topics      list of topics to record when record_all is false
  ~lz4         lz4-compress the bag (default true)
  ~stop_settle seconds to wait for a clean SIGINT exit before escalating (default 10)
"""
import os
import signal
import subprocess
import datetime
import rospy
from mavros_msgs.msg import State


class ArmRecorder(object):
    def __init__(self):
        self.bag_dir = os.path.expanduser(rospy.get_param("~bag_dir", "/work/flight_logs"))
        self.prefix = rospy.get_param("~prefix", "flight")
        self.record_all = bool(rospy.get_param("~record_all", False))
        self.topics = list(rospy.get_param("~topics", []))
        self.lz4 = bool(rospy.get_param("~lz4", True))
        self.settle = float(rospy.get_param("~stop_settle", 10.0))
        try:
            os.makedirs(self.bag_dir)
        except OSError:
            pass
        self.proc = None
        self.cur_bag = None
        self.armed = False
        if not self.record_all and not self.topics:
            rospy.logwarn("[arm_recorder] record_all=false but ~topics is empty -> nothing to record")
        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=5)
        rospy.on_shutdown(self._stop)
        rospy.loginfo("[arm_recorder] ready. ARM -> record %s into %s",
                      "-a (ALL topics)" if self.record_all else "%d topics" % len(self.topics),
                      self.bag_dir)

    def _on_state(self, m):
        if m.armed and not self.armed:
            self.armed = True
            self._start()
        elif not m.armed and self.armed:
            self.armed = False
            self._stop()

    def _start(self):
        if self.proc is not None and self.proc.poll() is None:
            rospy.logwarn("[arm_recorder] ARM but a recording is already running -> skip")
            return
        stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        self.cur_bag = os.path.join(self.bag_dir, "%s_%s.bag" % (self.prefix, stamp))
        cmd = ["rosbag", "record", "-O", self.cur_bag]
        if self.lz4:
            cmd.append("--lz4")
        cmd += (["-a"] if self.record_all else self.topics)
        # start in its own process group so our SIGINT goes to record, not the launcher
        self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        rospy.loginfo("[arm_recorder] ARMED -> recording -> %s (pid %d)", self.cur_bag, self.proc.pid)

    def _stop(self):
        if self.proc is None or self.proc.poll() is not None:
            self.proc = None
            return
        rospy.loginfo("[arm_recorder] DISARMED -> stopping recorder (SIGINT, clean finalize)…")
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except OSError:
            self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=self.settle)
            sz = os.path.getsize(self.cur_bag) / 1e6 if os.path.exists(self.cur_bag) else 0.0
            rospy.loginfo("[arm_recorder] STOPPED -> bag saved (%.1f MB): %s", sz, self.cur_bag)
        except subprocess.TimeoutExpired:
            rospy.logwarn("[arm_recorder] no clean exit in %.0fs -> SIGTERM/KILL (bag may be unindexed)", self.settle)
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None


def main():
    rospy.init_node("flight_safety_recorder")
    ArmRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
