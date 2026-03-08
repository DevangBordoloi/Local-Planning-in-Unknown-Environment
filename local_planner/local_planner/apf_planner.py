#!/usr/bin/env python3
"""
Artificial Potential Field (APF) Local Planner for TurtleBot3.

Uses the classic APF approach (Khatib 1986) with key improvements to
escape local-minima traps:

  1. **Tangential replacement** – When the repulsive force opposes the
     attractive force (local-minimum condition), the repulsive vector is
     *rotated* toward a 90-degree tangential direction rather than simply
     summed.  This eliminates the backward component that causes freezing
     (Borenstein & Koren 1991, improved).

  2. **Direction memory** – Once a tangential direction (CW / CCW) is
     selected, it is held for a minimum time window to prevent oscillation
     between the two directions.

  3. **Gap-based escape** – When the robot remains stuck despite tangential
     forces, a LIDAR-based gap finder identifies the clearest path toward
     the goal and steers through it.

References
----------
  O. Khatib, "Real-Time Obstacle Avoidance for Manipulators and Mobile
  Robots", Int. J. Robotics Research, 1986.

  J. Borenstein and Y. Koren, "Real-Time Obstacle Avoidance for Fast
  Mobile Robots in Cluttered Environments", IEEE Trans. SMC, 1991.
"""

import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class APFPlanner(Node):
    """ROS 2 node – APF planner with tangential-replacement escape."""

    # ── TurtleBot3 Burger limits ──
    MAX_LIN = 0.22   # m/s
    MAX_ANG = 2.84   # rad/s

    # ── APF gains ──
    K_ATT  = 1.2     # attractive gain
    K_REP  = 0.5     # repulsive gain
    D0     = 0.8     # obstacle influence radius [m]

    # ── Tangential ──
    K_TANG        = 1.2    # tangential gain
    TANG_MIN_FRAC = 0.30   # minimum tangential blend even when not opposed
    TANG_HOLD     = 40     # ticks to remember chosen direction (~4 s)

    # ── Velocity ──
    KP_V  = 0.8
    KP_W  = 2.5
    MIN_V = 0.02   # creep forward so robot never fully halts

    # ── Goal ──
    GOAL_TOL = 0.3

    # ── Stuck detection ──
    STUCK_WINDOW = 25    # ticks (~2.5 s at 10 Hz)
    STUCK_DISP   = 0.05  # [m]

    def __init__(self):
        super().__init__('apf_planner')

        self.declare_parameter('goal_x', 4.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.3)

        self.goal_x  = self.get_parameter('goal_x').value
        self.goal_y  = self.get_parameter('goal_y').value
        self.GOAL_TOL = self.get_parameter('goal_tolerance').value

        # Robot state
        self.x = self.y = self.yaw = 0.0
        self.odom_ok = False
        self.scan: LaserScan | None = None
        self.goal_reached = False

        # Metrics
        self.t0 = None
        self.min_obs = float('inf')
        self.total_dist = 0.0
        self.prev_xy = None

        # Navigation state
        self.pos_hist: list[tuple[float, float]] = []
        self.tang_dir = 0          # 0 = auto, +1 = CCW, -1 = CW
        self.tang_hold_cnt = 0     # ticks remaining to hold tang_dir
        self.escape_ticks = 0      # >0 ⇒ executing escape manoeuvre
        self.escape_ang = 0.0      # angular velocity during escape
        self.stuck_count = 0

        # Publishers
        self.cmd_pub   = self.create_publisher(Twist,  'cmd_vel', 10)
        self.force_pub = self.create_publisher(Marker, 'apf/force_vector', 10)
        self.att_pub   = self.create_publisher(Marker, 'apf/attractive', 10)
        self.rep_pub   = self.create_publisher(Marker, 'apf/repulsive', 10)
        self.goal_pub  = self.create_publisher(Marker, 'goal_marker', 10)
        self.path_pub  = self.create_publisher(Marker, 'path_trace', 10)
        self.path_points: list[Point] = []

        # Subscribers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, 'scan', self._cb_scan, qos)
        self.create_subscription(Odometry,  'odom', self._cb_odom, 10)

        self.create_timer(0.1, self._loop)
        self.get_logger().info(
            f'APF Planner started → goal ({self.goal_x}, {self.goal_y})')

    # ═══════════════════════════════════════════════════════════ callbacks
    def _cb_scan(self, msg: LaserScan):
        self.scan = msg

    def _cb_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                              1.0 - 2.0 * (o.y ** 2 + o.z ** 2))
        self.x, self.y = p.x, p.y
        self.odom_ok = True
        if self.prev_xy is not None:
            self.total_dist += math.hypot(p.x - self.prev_xy[0],
                                          p.y - self.prev_xy[1])
        self.prev_xy = (p.x, p.y)

    # ═══════════════════════════════════════════════════════════ forces
    def _attractive(self):
        """Conic (far) / quadratic (near) attractive potential."""
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        d = math.hypot(dx, dy)
        if d < 0.01:
            return 0.0, 0.0
        if d <= 1.0:
            return self.K_ATT * dx, self.K_ATT * dy
        return self.K_ATT * dx / d, self.K_ATT * dy / d

    def _repulsive_with_tangential(self, f_att):
        """Compute repulsive + tangential force with direction memory.

        Key idea – instead of *adding* the tangential to the repulsive,
        we *blend / replace*: when the repulsive directly opposes the
        attractive (opposition ≈ 1), the repulsive vector is rotated
        toward pure tangential.  This removes the backward component
        that causes oscillation.
        """
        scan = self.scan
        rx = ry = 0.0
        a = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max and r < self.D0:
                ox = self.x + r * math.cos(self.yaw + a)
                oy = self.y + r * math.sin(self.yaw + a)
                dx, dy = self.x - ox, self.y - oy
                d = max(math.hypot(dx, dy), 0.01)
                if d < self.min_obs:
                    self.min_obs = d
                mag = self.K_REP * (1.0 / d - 1.0 / self.D0) / (d * d)
                rx += mag * dx / d
                ry += mag * dy / d
            a += scan.angle_increment

        rmag = math.hypot(rx, ry)
        if rmag < 1e-6:
            # No obstacles → decay direction memory
            self.tang_hold_cnt = max(0, self.tang_hold_cnt - 1)
            if self.tang_hold_cnt <= 0:
                self.tang_dir = 0
            return 0.0, 0.0

        amag = math.hypot(f_att[0], f_att[1])
        if amag < 1e-6:
            return rx, ry

        # ── tangential candidates (±90° rotation of repulsive) ──
        tx_ccw, ty_ccw = -ry, rx      # counter-clockwise
        tx_cw,  ty_cw  =  ry, -rx     # clockwise

        # ── direction selection with memory ──
        if self.tang_hold_cnt > 0:
            # Keep previous direction
            if self.tang_dir > 0:
                tx, ty = tx_ccw, ty_ccw
            else:
                tx, ty = tx_cw, ty_cw
            self.tang_hold_cnt -= 1
        else:
            # Pick whichever tangential aligns better with attractive
            dot_ccw = tx_ccw * f_att[0] + ty_ccw * f_att[1]
            dot_cw  = tx_cw  * f_att[0] + ty_cw  * f_att[1]
            if dot_ccw >= dot_cw:
                tx, ty = tx_ccw, ty_ccw
                self.tang_dir = 1
            else:
                tx, ty = tx_cw, ty_cw
                self.tang_dir = -1
            self.tang_hold_cnt = self.TANG_HOLD

        # ── opposition: how much repulsive opposes attractive ──
        cos_ar = (f_att[0] * rx + f_att[1] * ry) / (amag * rmag)
        opposition = max(0.0, -cos_ar)          # 0 … 1

        # ── blend: REPLACE repulsive with tangential when opposed ──
        # α = 0 → pure repulsive,  α = 1 → pure tangential
        alpha = self.TANG_MIN_FRAC + (1.0 - self.TANG_MIN_FRAC) * opposition
        k = self.K_TANG * alpha
        out_x = rx * (1.0 - alpha) + k * tx
        out_y = ry * (1.0 - alpha) + k * ty
        return out_x, out_y

    # ═══════════════════════════════════════════════════════════ stuck
    def _check_stuck(self):
        self.pos_hist.append((self.x, self.y))
        if len(self.pos_hist) > self.STUCK_WINDOW:
            self.pos_hist.pop(0)
        if len(self.pos_hist) < self.STUCK_WINDOW:
            return False
        ox, oy = self.pos_hist[0]
        return math.hypot(self.x - ox, self.y - oy) < self.STUCK_DISP

    def _find_gap_angle(self):
        """Return the world-frame angle of the best LIDAR gap toward goal."""
        scan = self.scan
        if scan is None:
            return None
        ranges = np.array(scan.ranges)
        n = len(ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        # "Free" = range above a threshold (or inf / out-of-range)
        free = np.ones(n, dtype=bool)
        valid = (ranges > scan.range_min) & np.isfinite(ranges)
        free[valid] = ranges[valid] > 0.45
        # ranges that are inf / NaN / out-of-range → treated as free

        if not np.any(free):
            return None

        # Goal angle in robot frame
        gx, gy = self.goal_x - self.x, self.goal_y - self.y
        goal_robot = math.atan2(math.sin(math.atan2(gy, gx) - self.yaw),
                                math.cos(math.atan2(gy, gx) - self.yaw))

        # Score free rays by closeness to goal direction
        free_angles = angles[free]
        diffs = np.abs(np.arctan2(np.sin(free_angles - goal_robot),
                                   np.cos(free_angles - goal_robot)))
        best_idx = np.argmin(diffs)
        return float(free_angles[best_idx]) + self.yaw   # world frame

    # ═══════════════════════════════════════════════════════════ vel cmd
    def _force_to_cmd(self, fx, fy):
        mag = math.hypot(fx, fy)
        if mag < 1e-6:
            return Twist()
        desired = math.atan2(fy, fx)
        err = math.atan2(math.sin(desired - self.yaw),
                         math.cos(desired - self.yaw))
        v = self.KP_V * mag * max(0.0, math.cos(err))
        w = self.KP_W * err
        cmd = Twist()
        cmd.linear.x = min(max(v, self.MIN_V), self.MAX_LIN)
        cmd.angular.z = min(max(w, -self.MAX_ANG), self.MAX_ANG)
        # Turning sharply → slow down
        if abs(err) > math.radians(70):
            cmd.linear.x = self.MIN_V
        return cmd

    # ═══════════════════════════════════════════════════════════ main loop
    def _loop(self):
        if not self.odom_ok or self.scan is None:
            return
        if self.t0 is None:
            self.t0 = self.get_clock().now()

        dg = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        if dg < self.GOAL_TOL:
            if not self.goal_reached:
                t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
                self.goal_reached = True
                self.get_logger().info(
                    f'[APF] GOAL REACHED in {t:.2f}s | '
                    f'dist {self.total_dist:.2f}m | '
                    f'min obs {self.min_obs:.2f}m')
            self.cmd_pub.publish(Twist())
            return

        # ── active escape manoeuvre ──
        if self.escape_ticks > 0:
            cmd = Twist()
            if self.escape_ticks > 20:            # back up ~1 s
                cmd.linear.x = -0.12
                cmd.angular.z = 0.0
            else:                                  # steer toward gap ~2 s
                cmd.linear.x = 0.10
                cmd.angular.z = self.escape_ang
            self.escape_ticks -= 1
            self.cmd_pub.publish(cmd)
            self._vis(self._attractive(), (0.0, 0.0), self._attractive())
            return

        # ── compute forces ──
        f_att = self._attractive()
        f_rep = self._repulsive_with_tangential(f_att)
        f_tot = (f_att[0] + f_rep[0], f_att[1] + f_rep[1])

        # ── stuck? → gap-based escape ──
        if self._check_stuck():
            self.stuck_count += 1
            self.escape_ticks = 30               # 3 s total

            # Flip tangential direction so next attempt tries the other way
            if self.tang_dir != 0:
                self.tang_dir *= -1
                self.tang_hold_cnt = 60          # hold flipped direction

            # Steer toward best gap
            gap = self._find_gap_angle()
            if gap is not None:
                err = math.atan2(math.sin(gap - self.yaw),
                                 math.cos(gap - self.yaw))
                self.escape_ang = max(-1.8, min(1.8, 2.5 * err))
            else:
                self.escape_ang = 1.5 * (1 if random.random() > 0.5 else -1)

            self.pos_hist.clear()
            self.get_logger().info(
                f'[APF] Stuck #{self.stuck_count} → gap escape '
                f'(ang={self.escape_ang:.2f})',
                throttle_duration_sec=1.0)
            return

        # ── normal: follow resultant force ──
        cmd = self._force_to_cmd(*f_tot)
        self.cmd_pub.publish(cmd)
        self._vis(f_att, f_rep, f_tot)

    # ═══════════════════════════════════════════════════════════ visualization
    def _vis(self, f_att, f_rep, f_tot):
        self._arrow('apf/attractive',   f_att, ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9))
        self._arrow('apf/repulsive',    f_rep, ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9))
        self._arrow('apf/force_vector', f_tot, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
        self.path_points.append(Point(x=self.x, y=self.y, z=0.0))
        self._goal_marker()
        self._path_trace()

    def _arrow(self, topic, force, color):
        pubs = {'apf/attractive':   self.att_pub,
                'apf/repulsive':    self.rep_pub,
                'apf/force_vector': self.force_pub}
        pub = pubs.get(topic)
        if pub is None:
            return
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = topic.replace('/', '_')
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 0.03;  m.scale.y = 0.06;  m.scale.z = 0.06
        m.color = color
        s = Point(x=self.x, y=self.y, z=0.1)
        e = Point(x=self.x + force[0], y=self.y + force[1], z=0.1)
        m.points = [s, e]
        m.lifetime.nanosec = 200_000_000
        pub.publish(m)

    def _goal_marker(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'goal';  m.id = 0
        m.type = Marker.CYLINDER;  m.action = Marker.ADD
        m.pose.position = Point(x=self.goal_x, y=self.goal_y, z=0.25)
        m.scale.x = 0.5;  m.scale.y = 0.5;  m.scale.z = 0.5
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.85)
        self.goal_pub.publish(m)
        t = Marker()
        t.header.frame_id = 'odom'
        t.header.stamp = self.get_clock().now().to_msg()
        t.ns = 'goal_text';  t.id = 1
        t.type = Marker.TEXT_VIEW_FACING;  t.action = Marker.ADD
        t.pose.position = Point(x=self.goal_x, y=self.goal_y, z=0.8)
        t.scale.z = 0.3
        t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        t.text = 'GOAL'
        self.goal_pub.publish(t)

    def _path_trace(self):
        if len(self.path_points) < 2:
            return
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'path_trace';  m.id = 0
        m.type = Marker.LINE_STRIP;  m.action = Marker.ADD
        m.scale.x = 0.03
        m.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)
        m.points = list(self.path_points)
        self.path_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = APFPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
