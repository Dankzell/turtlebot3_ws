#!/usr/bin/python3
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from math import sin, cos, pi
from squaternion import Quaternion

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_msg_TEST')
        self.publisher_ = self.create_publisher(Path, '/waypoints', 10)
        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_callback()

    def timer_callback(self):

        self.get_logger().info('Publishing Path')

        waypoints = np.array([(0.5, 0.5), (1.0, 1.0), (1.5, 1.5), (0, 2.5), (0, 3), (0, 3.5), (0, 4), (0, 4.5), (0, 5), (0, 5.5), (0, 6), (1.5, 6), (3, 5), (5,2), (6, 1), (2,1)])

        msg = Path()
        msg.header.frame_id = "odom"

        for k in range(len(waypoints) - 1):
            pose = PoseStamped()
            pose.pose.position.x = waypoints[k,0]
            pose.pose.position.y = waypoints[k,1]
            pose.pose.position.z = 0.0

            # Calculate the angle to the next waypoint
            dx = waypoints[k+1, 0] - waypoints[k, 0]
            dy = waypoints[k+1, 1] - waypoints[k, 1]
            yaw = np.arctan2(dy, dx)

            # Convert the yaw angle to a quaternion
            quat = R.from_euler('z', yaw).as_quat()

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            msg.poses.append(pose)

        # Add the last waypoint with the same orientation as the second last waypoint
        pose = PoseStamped()
        pose.pose.position.x = waypoints[-1,0]
        pose.pose.position.y = waypoints[-1,1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        msg.poses.append(pose)

        for point in msg.poses:
            self.get_logger().info(f'Path point: x = {point.pose.position.x}, y = {point.pose.position.y}')

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Path')       


def main(args=None):
    rclpy.init(args=args)

    viz_publisher = PathPublisher()
    viz_publisher.get_logger().info("Path Publisher has been started")

    rclpy.spin(viz_publisher)

    viz_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()