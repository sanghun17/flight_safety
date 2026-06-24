"""IMU touchdown detector -- pure logic, no ROS deps (so the node AND the offline
bag-replay validator share the exact same code).

During an emergency LAND (response descending in place at ~1 m/s, vx=vy=0) the only
thing that spikes the IMU is hitting the ground. land_drill bags (2026-06-24) show:
  descent (in LAND) : world-vertical specific force vert <= ~10.7 m/s^2, |gyro| <= ~0.14
  touchdown         : vert 20..50 m/s^2 + |gyro| 0.7..1.6, in a 1-2 sample BURST
                      (then rebounds as the airframe unloads) -- NOT a sustained run.

Feature = WORLD-vertical specific force (vert), i.e. body accel rotated to ENU-up by
the EKF attitude. NOT |a| (a horizontal maneuver inflates it) and NOT raw body-z
(the airframe tips on contact -> the impact rotates out of body-z; measured body az
at impact was ~8.4, indistinguishable from descent). The vertical impact is 92-99%
of |a| in the bags, so vert keeps full detection while rejecting lateral accel.
Gyro is required too (AND): a real impact rocks the airframe; this rejects a lone
single-axis accel glitch. Burst-tolerant: latch on `confirm` hits within window_s.

Feed it (t, vert, gyro_mag, in_land); update() returns True ONCE, at first confirmed
touchdown. Latches until in_land goes false (disarm / lane change).
"""


class TouchdownDetector(object):
    def __init__(self, vert_thresh=16.0, gyro_thresh=0.3, min_land_s=0.3,
                 confirm=2, window_s=0.25):
        self.vert_thresh = float(vert_thresh)   # world-up specific force (m/s^2) for an impact "hit"
        self.gyro_thresh = float(gyro_thresh)   # also require |gyro| (rad/s) >= this; 0 = ignore
        self.min_land_s = float(min_land_s)     # ignore this long after LAND begins (entry transient)
        self.confirm = int(confirm)             # hits needed within window_s to latch
        self.window_s = float(window_s)
        self.land_t0 = None                     # when LAND began
        self.hits = []                          # timestamps of recent impact hits
        self.fired = False

    def update(self, t, vert, gyro_mag, in_land):
        if not in_land:                         # only armed during the emergency descent
            self.land_t0 = None
            self.hits = []
            self.fired = False
            return False
        if self.land_t0 is None:
            self.land_t0 = t
        if self.fired or (t - self.land_t0) < self.min_land_s:
            return False
        if vert > self.vert_thresh and gyro_mag >= self.gyro_thresh:
            self.hits.append(t)
            self.hits = [h for h in self.hits if t - h <= self.window_s]
            if len(self.hits) >= self.confirm:
                self.fired = True
                return True
        return False
