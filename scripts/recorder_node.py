#!/usr/bin/env python
"""Arm-triggered rosbag recorder.

Watches /mavros/state: on ARM (disarmed->armed) it spawns `rosbag record` of the
configured topics into ~bag_dir/<prefix>_<stamp>.bag; on DISARM it stops the
recorder with SIGINT so rosbag finalizes the index cleanly (NOT SIGKILL -> that
leaves an unindexed bag needing `rosbag reindex`). One bag per arming.

On the same ARM/DISARM edge it also triggers the webcam recorder over ROS: it
calls the std_srvs/Trigger services /recorder/{start,stop} (the webcam is now a
ROS node on the same master). Fire-and-forget in a daemon thread: an unavailable
service logs a warning but never blocks or fails the rosbag capture. (Trigger
takes no args, so the .mp4 keeps the recorder's own flight_<stamp> name.)

Params (set in launch/config/recorder.yaml):
  ~bag_dir     output dir (default /work/flight_logs)
  ~prefix      bag basename prefix (default "flight")
  ~record_all  true -> `rosbag record -a` (everything); else record ~topics
  ~topics      list of topics to record when record_all is false
  ~lz4         lz4-compress the bag (default true)
  ~stop_settle seconds to wait for a clean SIGINT exit before escalating (default 10)
  ~webcam_enable     also drive the webcam recorder on ARM/DISARM (default true)
  ~webcam_start_srv  Trigger service that starts it (default /recorder/start)
  ~webcam_stop_srv   Trigger service that stops it  (default /recorder/stop)
  ~webcam_timeout    wait_for_service timeout seconds (default 5)
"""
import os
import re
import signal
import subprocess
import datetime
import threading
import rospy
from mavros_msgs.msg import State
from std_srvs.srv import Trigger


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
        self.webcam_on = False
        self.webcam_enable = bool(rospy.get_param("~webcam_enable", True))
        self.webcam_start_srv = rospy.get_param("~webcam_start_srv", "/recorder/start")
        self.webcam_stop_srv = rospy.get_param("~webcam_stop_srv", "/recorder/stop")
        self.webcam_timeout = float(rospy.get_param("~webcam_timeout", 5.0))
        if self.webcam_enable:
            rospy.loginfo("[arm_recorder] webcam via ROS service %s / %s",
                          self.webcam_start_srv, self.webcam_stop_srv)
        if not self.record_all and not self.topics:
            rospy.logwarn("[arm_recorder] record_all=false but ~topics is empty -> nothing to record")
        self._recover_orphans()   # boot self-heal: re-queue bags whose recorder died before marking .ready
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
        self._webcam_start()

    def _webcam_start(self):
        if not self.webcam_enable or self.webcam_on:
            return
        self.webcam_on = True
        # fire-and-forget: starting the webcam must never delay or fail the ARM path
        t = threading.Thread(target=self._call_trigger, args=(self.webcam_start_srv, "start"))
        t.daemon = True
        t.start()

    def _webcam_stop(self):
        """Stop the webcam recorder synchronously; return its saved .mp4 path (or None)."""
        if not self.webcam_enable or not self.webcam_on:
            return None
        self.webcam_on = False
        resp = self._call_trigger(self.webcam_stop_srv, "stop")
        if resp is not None and resp.success:
            m = re.search(r"(\S+\.mp4)", resp.message or "")
            if m:
                return m.group(1)
        return None

    def _call_trigger(self, srv, what):
        try:
            rospy.wait_for_service(srv, timeout=self.webcam_timeout)
            resp = rospy.ServiceProxy(srv, Trigger)()
            (rospy.loginfo if resp.success else rospy.logwarn)(
                "[arm_recorder] webcam %s: %s", what, resp.message)
            return resp
        except Exception as e:
            rospy.logwarn("[arm_recorder] webcam %s failed (%s): %s", what, srv, e)
            return None

    def _stop(self):
        mp4 = self._webcam_stop()
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
        except subprocess.TimeoutExpired:
            rospy.logwarn("[arm_recorder] no clean exit in %.0fs -> SIGTERM/KILL (bag may be unindexed)", self.settle)
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
        self.cur_bag = self._match_bag_name(self.cur_bag, mp4)   # bag basename := webcam mp4 basename
        if self.cur_bag and os.path.exists(self.cur_bag):
            sz = os.path.getsize(self.cur_bag) / 1e6
            rospy.loginfo("[arm_recorder] STOPPED -> bag saved (%.1f MB): %s", sz, self.cur_bag)
            self._mark_ready(self.cur_bag)   # host-side watcher rsyncs bag(+extrinsic) to the ml PC

    def _match_bag_name(self, bag, mp4):
        """Rename the finalized bag to the webcam mp4's basename so .bag and .mp4 match."""
        if not bag or not mp4 or not os.path.exists(bag):
            return bag
        newbag = os.path.join(self.bag_dir, os.path.splitext(os.path.basename(mp4))[0] + ".bag")
        if newbag == bag:
            return bag
        try:
            os.rename(bag, newbag)
            rospy.loginfo("[arm_recorder] bag renamed to match webcam mp4 -> %s", newbag)
            return newbag
        except OSError as e:
            rospy.logwarn("[arm_recorder] bag rename failed (%s) -> keeping %s", e, bag)
            return bag

    def _mark_ready(self, bag):
        """Drop a <base>.ready marker so a host-side watcher knows the bag is final -> rsync it."""
        try:
            open(os.path.splitext(bag)[0] + ".ready", "w").close()
        except OSError as e:
            rospy.logwarn("[arm_recorder] could not write .ready marker: %s", e)

    def _recover_orphans(self):
        """Boot self-heal for the finalize->mark_ready gap. If a recorder is killed after the bag
        is written but before _mark_ready (Ctrl-C / stack restart / crash during the SIGINT finalize
        window), the bag survives (rosbag runs in its own setsid session) but has no .ready marker,
        so the host watcher never syncs it. On startup, re-queue any bag with NEITHER a .ready (still
        pending) NOR a .synced (watcher's done-breadcrumb, written on successful rsync). Idempotent."""
        try:
            bags = [f for f in os.listdir(self.bag_dir) if f.endswith(".bag")]
        except OSError as e:
            rospy.logwarn("[arm_recorder] orphan scan skipped (%s)", e)
            return
        n = 0
        for f in sorted(bags):
            base = os.path.join(self.bag_dir, f[:-len(".bag")])
            if os.path.exists(base + ".ready") or os.path.exists(base + ".synced"):
                continue
            self._mark_ready(base + ".bag")
            n += 1
            rospy.loginfo("[arm_recorder] orphan bag re-queued for sync: %s", f)
        if n:
            rospy.loginfo("[arm_recorder] boot self-heal: %d orphan bag(s) marked ready", n)


def main():
    rospy.init_node("flight_safety_recorder")
    ArmRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
