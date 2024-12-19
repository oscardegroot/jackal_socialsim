#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.time import Duration

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import rclpy.time
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import OperationModeState
from movebox.msg import MoveLatState

import os
import sys
import pathlib

path = pathlib.Path(__file__).parent.resolve()
sys.path.append(os.path.join(path))

from ros2_data_recorder import DataRecorderNode

class RobotStateMonitor(Node):
    def __init__(self):
        super().__init__('condition_check')

        # Get the parameters for the conditions
        self.condition_type = self.declare_parameter('condition_type', 'close_to_point').value  # 'close_to_point' or 'exceed_value'
        self.target_x = self.declare_parameter('target_x', 0.0).value
        self.target_y = self.declare_parameter('target_y', 0.0).value
        self.threshold_distance = self.declare_parameter('threshold_distance', 5.0).value
        self.max_x = self.declare_parameter('max_x', 10.0).value
        self.max_y = self.declare_parameter('max_y', 10.0).value
        self.timeout = self.declare_parameter('timeout', 10.0).value
        self.start_time = self.get_clock().now()
        self.num_experiments = self.declare_parameter('num_experiments', 1).value

        # Publisher for the reset topic
        self.reset_pub = self.create_publisher(Empty, '/jackal_socialsim/reset', 10)
        self.done_pub = self.create_publisher(Empty, '/jackal_socialsim/done', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 5)

        # Subscriber for the robot_state topic
        # self.state_sub = self.create_subscription(PoseStamped, 'robot_state', self.state_callback, 10)
        # self.state_sub = self.create_subscription(PoseStamped, '~/input/state', self.state_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/planning/mission_planning/goal', self.goal_callback, 10)
        self.state_sub = self.create_subscription(Odometry, '~/input/state', self.state_callback_odom, 10)
        self.autoware_sub = self.create_subscription(OperationModeState, '/api/operation_mode/state', self.autoware_status_callback, 10)
        self.subscription = self.create_subscription(MoveLatState,'/movebox/lat_state',
            self.movebox_callback,
            10)
        self.reset_sub = self.create_subscription(Empty, '~/input/reset', self.reset_callback, 1)
        self._external_reset = False
        self.is_simulation = self.declare_parameter('is_simulation', True).value

        self.completions = 0
        self.timeouts = 0
        self.first = True
        self.start_recording = False
        self.movebox_autonomous = False
        self.get_logger().info(f"Recording Data...")

    def set_data_recorder(self, data_recorder):
        self.data_recorder = data_recorder

    def state_callback_odom(self, msg):
        self.process_state(msg.pose.pose)

    def reset_callback(self, msg):
        pass
        # if self.is_simulation:
            # self.get_logger().info(f"External reset received (Simulation)")
            # self._external_reset = True

    def goal_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.get_logger().info("Goal received")

    def state_callback(self, msg):
        self.process_state(msg.pose)

    def autoware_status_callback(self, msg):
        if msg.mode == msg.AUTONOMOUS:
            self.start_recording = True
        else:
            self.start_recording = False

    def movebox_callback(self, msg):
        if msg.control_state_lat == 2:
            self.movebox_autonomous = True
        else:
            self.movebox_autonomous = False

    def process_state(self, msg):

        if self.start_recording and self.movebox_autonomous:
            self.data_recorder.enable_record()
        else:
            self.data_recorder.disable_record()
        
        if self.is_simulation:
            self.data_recorder.enable_record()
                
        x = msg.position.x
        y = msg.position.y

        should_reset = False
        elapsed_time = self.get_clock().now() - self.start_time

        if self.is_robot_flipped(msg) and elapsed_time > Duration(seconds=1.0):
            self.start_time = self.get_clock().now()  # Prevent double resets
            self.publish_reset()
            return

        if self.first:
            self.first = False
            self.publish_reset()  # Reset but without counting this one
            return

        if self.condition_type == 'close_to_point':
            # Condition 1: Check if the robot is close to a predefined point
            distance = math.sqrt((x - self.target_x) ** 2 + (y - self.target_y) ** 2)
            if distance <= self.threshold_distance:
                should_reset = True
        elif self.condition_type == 'exceed_value':
            # Condition 2: Check if both x and y exceed certain values
            if x >= self.max_x or y >= self.max_y:
                should_reset = True

        if self._external_reset and elapsed_time > Duration(seconds=5.0):
            should_reset = True
            self._external_reset = False
            self.get_logger().info("Experiment: External reset received")
        elif self._external_reset: # We already reset for another reason, ignore it
            self._external_reset = False

        if not should_reset:
            if self.is_simulation and elapsed_time > Duration(seconds=self.timeout):
                self.get_logger().info("Experiment: Timeout")
                should_reset = True
                self.timeouts += 1
                self.publish_timeout()
        
        if should_reset and elapsed_time > Duration(seconds=1.0):
            self.start_time = self.get_clock().now()  # Prevent double resets

            self.data_recorder.reset_callback(Empty())
            self.data_recorder.add_data()

            self.publish_reset()
            if self.completions == self.num_experiments-1:
                self.data_recorder.save_data()
                self.get_logger().info(f"{self.num_experiments} experiments completed!")
                self.publish_done()
                rclpy.shutdown()
            else:
                self.completions += 1
                self.get_logger().info(f"Experiment {self.completions}/{self.num_experiments}")

    def publish_reset(self):
        # Publish an empty message to reset the simulation
        self.get_logger().info(f"Published RESET Message")
        self.reset_pub.publish(Empty())
        self.publish_goal()
        # timer = self.create_rate(5)
        # for _ in range(5):
            # timer.sleep()
            # self.publish_goal()

        self.start_time = self.get_clock().now()

    def publish_done(self):
        self.get_logger().info(f"Published DONE message")
        self.done_pub.publish(Empty())

    def publish_timeout(self):
        self.data_recorder.timeout_callback(Empty())

    def publish_goal(self):
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = self.target_x
        goal_msg.pose.position.y = self.target_y
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        self.goal_pub.publish(goal_msg)

    def is_robot_flipped(self, pose):
        # Define the threshold (1/8 * pi)
        threshold = math.pi / 8

        # Check if the pitch or roll is greater than the threshold
        return abs(pose.orientation.x) > threshold or abs(pose.orientation.y) > threshold


def main(args=None):
    rclpy.init(args=args)

    monitor = RobotStateMonitor()
    data_recorder = DataRecorderNode(monitor)
    monitor.set_data_recorder(data_recorder)

    rclpy.spin(monitor)

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
