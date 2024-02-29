#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import math
import time

import rclpy

from geometry_msgs.msg import Twist

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile

from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import Patrol


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_server')


        self.goal = Patrol.Goal()


        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise servers
        self._action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info("Turtlebot3 patrol action server has been initialised.")


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request :)')
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        radius = self.goal.radius  # unit: m
        speed = 0.5  # unit: m/s

        feedback_msg = Patrol.Feedback()
        total_driving_time = 2 * math.pi * radius / speed
        feedback_msg.time_left = total_driving_time
        last_time = self.get_clock().now()

        # Start executing an action
        while (feedback_msg.time_left > 0):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Patrol.Result()

            curr_time = self.get_clock().now()
            duration = Duration()
            duration = (curr_time - last_time).nanoseconds / 1e9  # unit: s

            feedback_msg.time_left = total_driving_time - duration
            self.get_logger().info('Time left until the robot stops: {0}'.format(feedback_msg.time_left))
            goal_handle.publish_feedback(feedback_msg)

            # Give vel_cmd to Turtlebot3
            twist = Twist()
            twist = self.drive_circle(radius, speed)
            self.cmd_vel_pub.publish(twist)

            # Process rate
            time.sleep(0.010)  # unit: s

        # When the action is completed
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        goal_handle.succeed()
        result = Patrol.Result()
        result.success = True
        self.get_logger().info('Returning result: {0}'.format(result.success))

        return result

    def drive_circle(self, radius, velocity):
        self.twist = Twist()
        self.linear_velocity = velocity  # unit: m/s
        self.angular_velocity = self.linear_velocity / radius  # unit: rad/s

        self.twist.linear.x = self.linear_velocity
        self.twist.angular.z = self.angular_velocity

        return self.twist

def main(args=None):
    rclpy.init(args=args)

    patrol_action_server = Turtlebot3PatrolServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(patrol_action_server, executor=executor)

    patrol_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()