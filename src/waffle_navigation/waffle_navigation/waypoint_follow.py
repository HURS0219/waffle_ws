#!/usr/bin/env python3
"""Three-point auto navigation for the TurtleBot3 Waffle in the maze.

Reads three targets from a YAML file (map frame, anchored at the robot start
where slam_toolbox places the map origin) and drives the robot autonomously
through them with Navigation2 / nav2_waypoint_follower.

Usage:
    ros2 run waffle_navigation waypoint_follow.py
    # or with a custom waypoint file:
    ros2 run waffle_navigation waypoint_follow.py --ros-args -p config_file:=/path/to/waypoints.yaml
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def pose_from_xyyaw(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator('waypoint_follow')

    default_config = os.path.join(
        get_package_share_directory('waffle_navigation'), 'config', 'waypoints.yaml')
    nav.declare_parameter('config_file', default_config)
    config_file = nav.get_parameter('config_file').value
    nav.get_logger().info('加载航点配置: %s' % config_file)

    if not os.path.isfile(config_file):
        nav.get_logger().error('找不到航点文件: %s' % config_file)
        rclpy.shutdown()
        return

    with open(config_file, 'r') as f:
        cfg = yaml.safe_load(f)

    targets = cfg.get('targets', [])
    if len(targets) < 3:
        nav.get_logger().error('航点文件至少需要 3 个目标点 (targets)')
        rclpy.shutdown()
        return

    # SLAM-only flow: no AMCL node, so wait for the bt_navigator lifecycle node.
    nav.waitUntilNav2Active(navigator='bt_navigator', localizer='bt_navigator')

    waypoints = [pose_from_xyyaw(nav, p['x'], p['y'], p.get('yaw', 0.0))
                 for p in targets]

    nav.get_logger().info('开始三点自动导航, 共 %d 个目标点:' % len(waypoints))
    for i, p in enumerate(targets):
        nav.get_logger().info(
            '  目标%d: x=%.2f y=%.2f yaw=%.2f' % (i + 1, p['x'], p['y'], p.get('yaw', 0.0)))

    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback is not None and feedback.current_waypoint > 0:
            wp = feedback.current_waypoint
            rem = feedback.number_of_waypoints_remaining
            nav.get_logger().info(
                '正在前往目标%d, 剩余 %d 个目标点' % (wp + 1, rem),
                throttle_duration_sec=2.0)
        rclpy.spin_once(nav, timeout_sec=0.1)

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('任务完成: 三点自动导航全部成功')
    else:
        nav.get_logger().warn('任务结果: %s' % result)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
