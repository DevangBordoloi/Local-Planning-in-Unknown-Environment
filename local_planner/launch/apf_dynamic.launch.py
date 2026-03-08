#!/usr/bin/env python3
"""Launch Gazebo + TurtleBot3 + APF planner + dynamic obstacles (bonus)."""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('local_planner')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(pkg_dir, 'worlds', 'obstacle_world.world')
    tb3_launch_dir = os.path.join(tb3_gazebo_dir, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        DeclareLaunchArgument('goal_x', default_value='4.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch Gazebo GUI and RViz'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),

        # ---------- Gazebo server ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_file}.items(),
        ),

        # ---------- Gazebo client (optional) ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),

        # ---------- Robot state publisher ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir,
                             'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # ---------- Spawn TurtleBot3 ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose,
                              'y_pose': y_pose}.items(),
        ),

        # ---------- Dynamic obstacle manager ----------
        TimerAction(period=5.0, actions=[
            Node(
                package='local_planner',
                executable='dynamic_obstacle_manager',
                name='dynamic_obstacle_manager',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]),

        # ---------- APF planner node ----------
        Node(
            package='local_planner',
            executable='apf_planner',
            name='apf_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
            }],
        ),

        # ---------- RViz (optional) ----------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(gui),
            arguments=['-d', os.path.join(pkg_dir, 'rviz',
                                          'local_planner.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
