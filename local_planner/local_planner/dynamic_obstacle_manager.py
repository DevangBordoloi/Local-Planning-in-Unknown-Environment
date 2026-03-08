#!/usr/bin/env python3
"""
Dynamic obstacle spawner / mover for Gazebo.

Spawns box obstacles in the world and moves them back and forth to simulate
a dynamic environment.  Uses the gazebo_msgs SetEntityState service.
"""

import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist


# ---------- Obstacle definitions ----------
DYNAMIC_OBSTACLES = [
    {
        'name': 'dyn_obs_1',
        'size': [0.3, 0.3, 0.5],
        'path': [  # crosses path vertically at x=0.0
            (0.0, 2.0),
            (0.0, -2.0),
        ],
        'speed': 0.25,   # m/s
    },
    {
        'name': 'dyn_obs_2',
        'size': [0.4, 0.4, 0.5],
        'path': [  # crosses path vertically at x=2.0
            (2.0, 2.0),
            (2.0, -2.0),
        ],
        'speed': 0.20,
    },
    {
        'name': 'dyn_obs_3',
        'size': [0.35, 0.35, 0.5],
        'path': [  # crosses path diagonally
            (3.0, 1.5),
            (3.0, -1.5),
        ],
        'speed': 0.30,
    },
]

# SDF template for a dynamic box
BOX_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>5.0</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
        <material>
          <ambient>1.0 0.4 0.0 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class DynamicObstacleManager(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_manager')

        # Service clients
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

        # Try both possible service names for SetEntityState
        # (depends on gazebo_ros version)
        self.set_state_cli = None
        for srv_name in ['/set_entity_state', '/gazebo/set_entity_state']:
            cli = self.create_client(SetEntityState, srv_name)
            if cli.wait_for_service(timeout_sec=2.0):
                self.set_state_cli = cli
                self.get_logger().info(
                    f'Using SetEntityState service: {srv_name}')
                break

        self.get_logger().info('Waiting for spawn_entity service …')
        self.spawn_cli.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('Gazebo services available.')

        if self.set_state_cli is None:
            self.get_logger().warn(
                'SetEntityState service not found, '
                'trying /set_entity_state anyway.')
            self.set_state_cli = self.create_client(
                SetEntityState, '/set_entity_state')

        # Spawn obstacles
        self.obstacles = []
        for obs_def in DYNAMIC_OBSTACLES:
            self._spawn_obstacle(obs_def)
            self.obstacles.append({
                'name': obs_def['name'],
                'path': obs_def['path'],
                'speed': obs_def['speed'],
                'segment': 0,      # current path segment
                'forward': True,   # direction along path
                'pos': list(obs_def['path'][0]),
            })

        # Timer to move obstacles
        self.timer = self.create_timer(0.05, self.move_obstacles)
        self.get_logger().info(
            f'Spawned {len(self.obstacles)} dynamic obstacles.')

    # ------------------------------------------------------------------ #
    def _spawn_obstacle(self, obs_def):
        req = SpawnEntity.Request()
        req.name = obs_def['name']
        req.xml = BOX_SDF_TEMPLATE.format(
            name=obs_def['name'],
            sx=obs_def['size'][0],
            sy=obs_def['size'][1],
            sz=obs_def['size'][2],
        )
        req.initial_pose = Pose(
            position=Point(x=obs_def['path'][0][0],
                           y=obs_def['path'][0][1], z=0.25))
        req.reference_frame = 'world'
        future = self.spawn_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Spawned {obs_def['name']}: {f.result().status_message}"
                if f.result() else f"Spawn failed: {f.exception()}"))

    # ------------------------------------------------------------------ #
    def move_obstacles(self):
        dt = 0.05
        for obs in self.obstacles:
            path = obs['path']
            seg = obs['segment']
            target = path[seg + 1] if obs['forward'] else path[seg]
            start = path[seg] if obs['forward'] else path[seg + 1]

            dx = target[0] - obs['pos'][0]
            dy = target[1] - obs['pos'][1]
            dist = math.hypot(dx, dy)

            if dist < 0.05:
                # Switch direction or advance segment
                if obs['forward']:
                    if seg + 2 < len(path):
                        obs['segment'] += 1
                    else:
                        obs['forward'] = False
                else:
                    if seg > 0:
                        obs['segment'] -= 1
                    else:
                        obs['forward'] = True
                continue

            vx = obs['speed'] * dx / dist
            vy = obs['speed'] * dy / dist
            obs['pos'][0] += vx * dt
            obs['pos'][1] += vy * dt

            # Set state in Gazebo
            state = EntityState()
            state.name = obs['name']
            state.pose = Pose(
                position=Point(x=obs['pos'][0], y=obs['pos'][1], z=0.25),
                orientation=Quaternion(w=1.0))
            state.twist = Twist()

            req = SetEntityState.Request()
            req.state = state
            self.set_state_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
