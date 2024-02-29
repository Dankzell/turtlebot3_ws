import re
from typing import List, Tuple
import rclpy 
import math
import time
import numpy as np

from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path

from gustav_custom_interfaces.action import DriveTo

class PurePursuitActionClient(Node):

    def __init__(self):

        super().__init__('pure_pursuit_action_client')
        self._action_client = ActionClient(
            self, 
            DriveTo, 
            'drive_to')
        
    def send_goal(self):
        goal_msg = DriveTo.Goal()
        goal_msg.path = self.get_path()

        self._action_client.wait_for_server()

        self._send_goal_future =  self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))
        rclpy.shutdown()

    

    def get_path(self):
            pose1 = PoseStamped()
            pose2 = PoseStamped()
            pose3 = PoseStamped()
            pose4 = PoseStamped()
            pose5 = PoseStamped()
            pose6 = PoseStamped()

            pose1.pose.position.x = np.float64(2)
            pose1.pose.position.y = np.float64(0)

            pose2.pose.position.x = np.float64(3)
            pose2.pose.position.y = np.float64(0)

            pose3.pose.position.x = np.float64(4)
            pose3.pose.position.y = np.float64(0)

            pose4.pose.position.x = np.float64(5)
            pose4.pose.position.y = np.float64(0)

            pose5.pose.position.x = np.float64(6)
            pose5.pose.position.y = np.float64(0)

            pose6.pose.position.x = np.float64(6)
            pose6.pose.position.y = np.float64(3)

            path = Path()
            path.header.frame_id = 'odom'
            path.header.stamp = rclpy.clock.Clock().now().to_msg()
            path.poses.append(pose1)
            path.poses.append(pose2)

            path.poses.append(pose3)
            path.poses.append(pose4)
            path.poses.append(pose5)
            path.poses.append(pose6)
            return path
    

def main(args=None):
    rclpy.init(args=args)

    action_client = PurePursuitActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()    
