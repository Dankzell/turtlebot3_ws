import rclpy
import math

import numpy as np
from rclpy.node import Node
# from slam.slam.drive import Drive

from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Point, Twist, Pose
from nav_msgs.msg import Odometry, Path
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.action import CancelResponse

# from std_msgs.msg import Int16MultiArray, Float32MultiArray

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from gustav_custom_interfaces.action import DriveTo

"""
math function

delta = atan(2Lsin(alpha)/d)

d: distance between middle of robot to TP (look ahead distance)
L: wheelbase
R: distance between ICR and TP (turning radius?)

Ideas for imporvement:
- Not use np for the array but the path direclty which should make the program perform 
faster



"""

# Constants 
LOOKAHEAD = 2
WB = 0.3
VELOCITY = 0.5
GOAL_TOLERANCE = 0.5

class PurePursuitActionServer(Node):

    def __init__(self):
        super().__init__('pure_pursuit_action_server')

        self.rx = 0
        self.ry = 0
        self.yaw = 0
        self.waypoints = np.zeros((1,2))

        self.feedback_msg = None

        self.goal = DriveTo.Goal()
        self.feedback_msg = DriveTo.Feedback()

        callbackgroup = ReentrantCallbackGroup()

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoints_pub = self.create_publisher(Point, '/waypoints', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10, callback_group=callbackgroup)

        #init server
        self._action_server = ActionServer(
            self, 
            DriveTo,
            'drive_to',
            execute_callback=self.pure_pursuit,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
    
        self.get_logger().info('Pure Pursuit Action Server has been initiated')


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        self.goal = goal_request
        self.path_callback(self.goal.path)
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def pose_callback(self, input:Odometry):
        msg = input.pose
        self.feedback_msg.pose = msg.pose

        self.rx = msg.pose.position.x
        self.ry = msg.pose.position.y

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        quaternion = (qx, qy, qz, qw)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
    
    def path_callback(self, path):
        self.get_logger().info("Got a path")
        for pose in path.poses:
            tmp = np.array([pose.pose.position.x, pose.pose.position.y])
            if self.waypoints.shape == (1, 2):
                self.waypoints = tmp
            self.waypoints = np.vstack((self.waypoints, tmp))

    def find_distance(self, p0, p1):
        p0 = p0.astype(float)
        p1 = p1.astype(float)
        return np.sqrt(np.sum((p1 - p0)**2))

    def find_distance_index(self, idx, current_xy):
        distance = self.find_distance(current_xy, self.waypoints[idx])
        return distance

    #checks which of the waypoints is the closest to the current position
    def find_nearest_waypoint(self, current_xy):
        distance = self.find_distance(current_xy, self.waypoints[0])
        nearest_idx = 0
        for i in range(len(self.waypoints)):
            if self.find_distance(current_xy, self.waypoints[i]) < distance:
                distance = self.find_distance(current_xy, self.waypoints[i])
                nearest_idx = i
        return nearest_idx

    #checks if the waypoint is within lookahead distance
    def idx_close_to_lookahead(self,idx, current_xy):
        while self.find_distance_index(idx, current_xy) < LOOKAHEAD and idx < len(self.waypoints) - 1:
            idx += 1
        if idx == 0:
            return idx
        return idx - 1

    def pure_pursuit(self, goal_handle):
        self.get_logger().info('Pure pursuit started')
        curr_pose = np.array([self.rx, self.ry])

        msg = Twist()

        try:
            while self.find_distance(curr_pose, self.waypoints[-1]) > GOAL_TOLERANCE:
                self.get_logger().info('Current pose: ' + str(curr_pose))
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result = DriveTo.Result()
                    result.success = False
                    return result
                #self.robot_pose()
                curr_pose = np.array([self.rx, self.ry])

                nearest_idx = self.find_nearest_waypoint(curr_pose)
                lookahead_idx = self.idx_close_to_lookahead(nearest_idx, curr_pose)

                goal_pose = self.waypoints[lookahead_idx]

                ## Pure pursuit controller

                alpha = math.atan2(goal_pose[1] - self.ry, goal_pose[0] - self.rx) - self.yaw
                alpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Ensure alpha is in [-pi, pi]
                
                tmp_lookahead = self.find_distance(goal_pose, curr_pose)
                steering_angle = math.atan2(2 * WB * math.sin(alpha), tmp_lookahead)

                if steering_angle > 0.5:
                    steering_angle = 0.5
                elif steering_angle < -0.5:
                    steering_angle = -0.5

                # Publish messages
                msg.linear.x = VELOCITY
                msg.angular.z = steering_angle
                
                self.publisher.publish(msg)

        except IndexError:
            self.get_logger().info('Path completed')

        self.get_logger().info('Pure pursuit completed')

        msg.linear.x = 0.0
        msg.angular.z = 0.0
                
        self.publisher.publish(msg)

        goal_handle.succeed()
        result = DriveTo.Result()
        result.success = True
        return result
        
def main():
    rclpy.init()
    node = PurePursuitActionServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()