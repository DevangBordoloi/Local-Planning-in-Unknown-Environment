#!/usr/bin/env python3
"""
Dynamic Window Approach (DWA) Local Planner for TurtleBot3.

The DWA algorithm samples velocities in the robot's dynamic window (the set of
velocities reachable within one time-step given acceleration limits) and
evaluates each candidate trajectory using a cost function that balances:
  1. Progress towards the goal heading
  2. Clearance from obstacles
  3. Forward velocity (prefer faster motion)

Reference: Fox, Burgard & Thrun, "The Dynamic Window Approach to Collision
Avoidance", IEEE Robotics & Automation Magazine, 1997.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class DWAPlanner(Node):
    """ROS 2 node implementing the Dynamic Window Approach."""

    # ---------- TurtleBot3 Burger kinematic limits ----------
    MAX_LINEAR_VEL = 0.22        # m/s
    MIN_LINEAR_VEL = -0.05       # m/s  (slight reverse for recovery)
    MAX_ANGULAR_VEL = 2.84       # rad/s
    MAX_LINEAR_ACC = 2.5         # m/s²
    MAX_ANGULAR_ACC = 30.0       # rad/s² (large → effectively full range)

    # ---------- DWA parameters ----------
    DT = 0.1                     # time resolution of trajectory  [s]
    PREDICT_TIME = 0.8           # how far ahead to simulate      [s]
    V_RESOLUTION = 0.02          # linear velocity sample step    [m/s]
    W_RESOLUTION = 0.1           # angular velocity sample step   [rad/s]

    # Cost weights
    HEADING_WEIGHT = 1.5
    OBSTACLE_WEIGHT = 0.5
    VELOCITY_WEIGHT = 0.3
    DISTANCE_WEIGHT = 1.2        # prefer trajectories closer to goal
    OBSTACLE_RADIUS = 0.08       # safety margin around robot     [m]

    def __init__(self):
        super().__init__('dwa_planner')

        # Declare parameters
        self.declare_parameter('goal_x', 4.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.3)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # Robot state: [x, y, yaw, v, w]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.odom_received = False
        self.scan_data = None
        self.goal_reached = False

        # Metrics
        self.start_time = None
        self.min_obstacle_dist_overall = float('inf')
        self.total_distance = 0.0
        self.prev_pos = None

        # Recovery state
        self.stuck_count = 0

        # ---- Publishers ----
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.traj_pub = self.create_publisher(MarkerArray, 'dwa/trajectories', 10)
        self.best_traj_pub = self.create_publisher(Marker, 'dwa/best_trajectory', 10)
        self.goal_pub = self.create_publisher(Marker, 'goal_marker', 10)
        self.path_pub = self.create_publisher(Marker, 'path_trace', 10)

        # Path trace storage
        self.path_points = []

        # ---- Subscribers ----
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # ---- Main timer ----
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'DWA Planner started → goal ({self.goal_x}, {self.goal_y})')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def scan_callback(self, msg: LaserScan):
        self.scan_data = msg

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # Convert quaternion → yaw
        siny_cosp = 2.0 * (orientation.w * orientation.z +
                           orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.state = np.array([
            pos.x, pos.y, yaw,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        ])
        self.odom_received = True

        # Track distance
        if self.prev_pos is not None:
            self.total_distance += math.hypot(
                pos.x - self.prev_pos[0], pos.y - self.prev_pos[1])
        self.prev_pos = (pos.x, pos.y)

    # ------------------------------------------------------------------ #
    #  Control loop                                                       #
    # ------------------------------------------------------------------ #
    def control_loop(self):
        if not self.odom_received or self.scan_data is None:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        # Check goal reached
        dx = self.goal_x - self.state[0]
        dy = self.goal_y - self.state[1]
        dist_to_goal = math.hypot(dx, dy)
        if dist_to_goal < self.goal_tolerance:
            if not self.goal_reached:
                elapsed = (self.get_clock().now() -
                           self.start_time).nanoseconds / 1e9
                self.goal_reached = True
                self.get_logger().info(
                    f'[DWA] GOAL REACHED in {elapsed:.2f}s | '
                    f'distance travelled {self.total_distance:.2f}m | '
                    f'min obstacle dist {self.min_obstacle_dist_overall:.2f}m')
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        # Build obstacle list from laser scan
        obs_np = self._scan_to_obstacles()

        # Pre-emptive obstacle avoidance: if closest obstacle is very near,
        # slow down and turn away before DWA gets stuck
        min_front_range = self._min_front_range()
        if min_front_range < 0.35:
            # Too close — turn away from nearest obstacle
            cmd = Twist()
            cmd.linear.x = -0.03
            # Find which side is more clear
            left_clear = self._side_clearance(left=True)
            right_clear = self._side_clearance(left=False)
            cmd.angular.z = 1.5 if left_clear >= right_clear else -1.5
            self.cmd_pub.publish(cmd)
            self.path_points.append(
                Point(x=self.state[0], y=self.state[1], z=0.0))
            self._publish_goal_marker()
            self._publish_path_trace()
            return

        # DWA core
        best_v, best_w, best_traj, all_trajs = self._dwa_control(obs_np)

        # Publish command
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)

        # Record path
        self.path_points.append(
            Point(x=self.state[0], y=self.state[1], z=0.0))

        # Publish visualisation
        self._publish_trajectories(all_trajs, best_traj)
        self._publish_goal_marker()
        self._publish_path_trace()

    # ------------------------------------------------------------------ #
    #  DWA core algorithm                                                 #
    # ------------------------------------------------------------------ #
    def _dwa_control(self, obs_np):
        """Evaluate all admissible (v, w) pairs and pick the best."""
        dw = self._dynamic_window()
        best_cost = -float('inf')
        best_v, best_w = 0.0, 0.0
        best_traj = None
        all_trajs = []

        for v in np.arange(dw[0], dw[1] + self.V_RESOLUTION, self.V_RESOLUTION):
            for w in np.arange(dw[2], dw[3] + self.W_RESOLUTION, self.W_RESOLUTION):
                traj = self._simulate_trajectory(v, w)
                heading_cost = self._heading_cost(traj)
                obs_cost = self._obstacle_cost(traj, obs_np)
                vel_cost = abs(v) / self.MAX_LINEAR_VEL
                dist_cost = self._distance_cost(traj)

                if obs_cost == -float('inf'):
                    # trajectory collides
                    all_trajs.append((traj, False))
                    continue

                cost = (self.HEADING_WEIGHT * heading_cost +
                        self.OBSTACLE_WEIGHT * obs_cost +
                        self.VELOCITY_WEIGHT * vel_cost +
                        self.DISTANCE_WEIGHT * dist_cost)

                all_trajs.append((traj, True))
                if cost > best_cost:
                    best_cost = cost
                    best_v, best_w = v, w
                    best_traj = traj

        if best_traj is None:
            # Phased recovery behavior
            self.stuck_count += 1
            goal_angle = math.atan2(
                self.goal_y - self.state[1],
                self.goal_x - self.state[0])
            yaw_err = goal_angle - self.state[2]
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            phase = self.stuck_count % 6
            if phase < 2:
                # Phase 1: back up to create clearance
                best_v = -0.05
                best_w = 0.0
            elif phase < 5:
                # Phase 2: rotate toward goal
                best_v = 0.0
                best_w = 2.0 if yaw_err >= 0 else -2.0
            else:
                # Phase 3: try going forward
                best_v = 0.1
                best_w = 1.5 if yaw_err >= 0 else -1.5

            best_traj = self._simulate_trajectory(best_v, best_w)
            self.get_logger().warn(
                f'No safe trajectory — recovery phase {phase} '
                f'(stuck count: {self.stuck_count})',
                throttle_duration_sec=3.0)
        else:
            if self.stuck_count > 0:
                self.stuck_count = max(0, self.stuck_count - 1)

        return best_v, best_w, best_traj, all_trajs

    def _dynamic_window(self):
        """Compute the dynamic window based on current velocity + acc limits."""
        v, w = self.state[3], self.state[4]
        vs = [self.MIN_LINEAR_VEL, self.MAX_LINEAR_VEL,
              -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL]
        vd = [v - self.MAX_LINEAR_ACC * self.DT,
              v + self.MAX_LINEAR_ACC * self.DT,
              w - self.MAX_ANGULAR_ACC * self.DT,
              w + self.MAX_ANGULAR_ACC * self.DT]
        return [max(vs[0], vd[0]), min(vs[1], vd[1]),
                max(vs[2], vd[2]), min(vs[3], vd[3])]

    def _simulate_trajectory(self, v, w):
        """Forward-simulate a trajectory with constant (v, w)."""
        x, y, yaw = self.state[0], self.state[1], self.state[2]
        traj = [(x, y, yaw)]
        for _ in range(int(self.PREDICT_TIME / self.DT)):
            x += v * math.cos(yaw) * self.DT
            y += v * math.sin(yaw) * self.DT
            yaw += w * self.DT
            traj.append((x, y, yaw))
        return traj

    def _heading_cost(self, traj):
        """Cost: alignment of trajectory endpoint with goal direction."""
        last = traj[-1]
        angle_to_goal = math.atan2(
            self.goal_y - last[1], self.goal_x - last[0])
        heading_err = abs(angle_to_goal - last[2])
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
        return 1.0 - abs(heading_err) / math.pi   # normalised [0, 1]

    def _distance_cost(self, traj):
        """Cost: how much closer the trajectory endpoint gets to the goal."""
        last = traj[-1]
        current_dist = math.hypot(
            self.goal_x - self.state[0], self.goal_y - self.state[1])
        end_dist = math.hypot(
            self.goal_x - last[0], self.goal_y - last[1])
        # Positive if trajectory gets closer, negative if farther
        progress = (current_dist - end_dist)
        return max(-1.0, min(progress / 0.3, 1.0))  # normalise to ~[-1, 1]

    def _obstacle_cost(self, traj, obs_np):
        """Cost: minimum distance to any obstacle along trajectory.
        obs_np is a numpy array of shape (N, 2)."""
        if obs_np is None or len(obs_np) == 0:
            return 1.0
        # Only check every other trajectory point to be less strict
        traj_pts = np.array([(tx, ty) for tx, ty, _ in traj[::2]])  # (T/2, 2)
        # Vectorised distance: (T, 1, 2) - (1, N, 2) → (T, N)
        diff = traj_pts[:, np.newaxis, :] - obs_np[np.newaxis, :, :]
        dists = np.sqrt((diff ** 2).sum(axis=2))
        min_dist = float(dists.min())
        if min_dist <= self.OBSTACLE_RADIUS:
            return -float('inf')   # collision
        # Track metric
        if min_dist < self.min_obstacle_dist_overall:
            self.min_obstacle_dist_overall = min_dist
        return min(min_dist / 2.0, 1.0)   # saturate at 2 m → 1.0

    # ------------------------------------------------------------------ #
    #  Helpers                                                            #
    # ------------------------------------------------------------------ #
    def _min_front_range(self):
        """Return the minimum LIDAR range in the front ±30° arc."""
        scan = self.scan_data
        n = len(scan.ranges)
        # Front arc: indices near 0 and near n (wrapping)
        arc_deg = 30
        arc_rays = int(arc_deg / math.degrees(scan.angle_increment))
        front_ranges = []
        for i in list(range(0, min(arc_rays, n))) + \
                 list(range(max(0, n - arc_rays), n)):
            r = scan.ranges[i]
            if scan.range_min < r < scan.range_max:
                front_ranges.append(r)
        return min(front_ranges) if front_ranges else 999.0

    def _side_clearance(self, left=True):
        """Return avg LIDAR range on one side (left=90°, right=270°)."""
        scan = self.scan_data
        n = len(scan.ranges)
        if left:
            center_idx = n // 4  # ~90°
        else:
            center_idx = 3 * n // 4  # ~270°
        arc = n // 12  # ±15°
        ranges = []
        for i in range(max(0, center_idx - arc),
                       min(n, center_idx + arc)):
            r = scan.ranges[i]
            if scan.range_min < r < scan.range_max:
                ranges.append(r)
        return sum(ranges) / len(ranges) if ranges else 0.0

    def _scan_to_obstacles(self):
        """Convert LaserScan → numpy array (N, 2) of obstacle points in odom."""
        scan = self.scan_data
        x, y, yaw = self.state[0], self.state[1], self.state[2]
        ranges = np.array(scan.ranges)
        angles = np.arange(len(ranges)) * scan.angle_increment + scan.angle_min
        # Filter valid and nearby
        valid = (ranges > scan.range_min) & (ranges < scan.range_max) & (ranges < 1.5)
        r_valid = ranges[valid]
        a_valid = angles[valid]
        if len(r_valid) == 0:
            return None
        # Subsample to max 90 points
        if len(r_valid) > 90:
            idx = np.linspace(0, len(r_valid) - 1, 90, dtype=int)
            r_valid = r_valid[idx]
            a_valid = a_valid[idx]
        ox = x + r_valid * np.cos(yaw + a_valid)
        oy = y + r_valid * np.sin(yaw + a_valid)
        return np.column_stack((ox, oy))

    # ------------------------------------------------------------------ #
    #  Visualisation                                                      #
    # ------------------------------------------------------------------ #
    def _publish_trajectories(self, all_trajs, best_traj):
        ma = MarkerArray()
        for idx, (traj, safe) in enumerate(all_trajs[:200]):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'dwa_candidates'
            m.id = idx
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.005
            colour = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3) if safe \
                else ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.15)
            m.color = colour
            for (tx, ty, _) in traj:
                m.points.append(Point(x=tx, y=ty, z=0.0))
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200_000_000
            ma.markers.append(m)
        self.traj_pub.publish(ma)

        if best_traj is not None:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'dwa_best'
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            for (tx, ty, _) in best_traj:
                m.points.append(Point(x=tx, y=ty, z=0.0))
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200_000_000
            self.best_traj_pub.publish(m)

    def _publish_goal_marker(self):
        """Publish a sphere marker at the goal location."""
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'goal'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position = Point(x=self.goal_x, y=self.goal_y, z=0.25)
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.85)
        self.goal_pub.publish(m)

        # Also publish a text label above the goal
        t = Marker()
        t.header.frame_id = 'odom'
        t.header.stamp = self.get_clock().now().to_msg()
        t.ns = 'goal_text'
        t.id = 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position = Point(x=self.goal_x, y=self.goal_y, z=0.8)
        t.scale.z = 0.3
        t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        t.text = 'GOAL'
        self.goal_pub.publish(t)

    def _publish_path_trace(self):
        """Publish the robot's actual path as a line strip."""
        if len(self.path_points) < 2:
            return
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'path_trace'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)
        m.points = list(self.path_points)
        self.path_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cmd = Twist()
            node.cmd_pub.publish(cmd)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
