#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
import pathlib
import copy
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mpc_planner_msgs.msg import ObstacleArray  # Replace `your_package` with your actual package name
from std_msgs.msg import String, Empty, Float64

# import tf_transformations  # ROS2 tf package
from geometry_msgs.msg import Quaternion

import threading


def quaternion_to_yaw(quaternion):
    # Convert the quaternion to a list [x, y, z, w]
    # q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    
    # Convert the quaternion to roll, pitch, and yaw
    yaw = math.atan2(2 * (quaternion.y * quaternion.w + quaternion.x * quaternion.z), \
                     1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

#     _, _, yaw = tf_transformations.euler_from_quaternion(q)
    
    # Return the yaw angle
    return yaw


class RecordedData:

    def __init__(self, save_interval_s):
        self._data = dict()
        self._save_interval_s = save_interval_s

        self._iteration = 0
        self.register("reset")
        self.register_metric("duration")

    def register(self, name):
        self._data[name] = []
    
    def register_metric(self, name):
        self._data[f"metric_{name}"] = []

    def add_metric(self, name, value):
        self._data[f"metric_{name}"].append(value)

    def add_singular_data(self, name, value):
        self._data[name] = value

    def add(self, name, value):
        if name not in self._data.keys():
            raise IOError("Trying to save unregistered data")
        
        self._data[name].append(value)
    
    def new_iteration(self):
        self._iteration += 1

    def end_experiment(self):
        self.add("reset", self._iteration)
        if len(self._data["reset"]) > 1:
            self.add_metric("duration", (self._data["reset"][-1] - self._data["reset"][-2]) * self._save_interval_s)
        else:
            self.add_metric("duration", -1)

    def has_data(self, name):
        return name in self._data.keys()

    def length(self) -> int:
        return self.get_iteration()

    def get_iteration(self) -> int:
        return self._iteration
    
    def get_data(self) -> dict:
        return self._data


class DataRecorderNode():
    def __init__(self, node):
        # super().__init__('data_recorder_node')
        self._node = node

        # Get the scenario name from the parameter
        self.scenario = node.declare_parameter('scenario', 'none').get_parameter_value().string_value
        self.experiment = node.declare_parameter('experiment', 'none').get_parameter_value().string_value
        self._data_folder = node.declare_parameter('data_folder', '').get_parameter_value().string_value

        self.save_interval_s = 50. / 1000.  # Every 50 ms
        self.num_experiments = node.declare_parameter('/condition_check/num_experiments', 1).get_parameter_value().integer_value

        # Initialize storage for the topics
        self._data = RecordedData(self.save_interval_s)

        self._data.add_singular_data("scenario", self.scenario)
        self._data.add_singular_data("experiment", self.experiment)

        self._reset = False
        self._state_msg = None
        self._twist_msg = None
        self._data.register("pos")
        self._data.register("v")
        self._data.register("orientation")

        self._collision_msg = None
        self._collisions = 0
        self._data.register_metric("collisions")

        self._timeout_received = False
        self._data.register_metric("timeout")

        self._obs_msg = None  # Note: registered at first occurrence
        self._num_obstacles = 0

        self._weights_msg = None
        # self._data.register("weights")

        # Subscribers
        node.create_subscription(Empty, '/jackal_socialsim/reset', self.reset_callback, 10)
        node.create_subscription(Empty, '/condition_check/timeout', self.timeout_callback, 10)
        # self.create_subscription(PoseStamped, '~/input/state', self.robot_state_callback, 10)
        node.create_subscription(Odometry, '~/input/state', self.robot_state_odom_callback, 10)
        node.create_subscription(Float64, 'pedestrian_simulator/collision_detected', self.collision_callback, 10)
        node.create_subscription(ObstacleArray, '/pedestrian_simulator/trajectory_predictions', self.obstacle_callback, 10)
        # self.create_subscription(WeightArray, 'hey_robot/weights', self.weights_callback, 10)

        # Timer for saving data
        self._save_lock = threading.Lock()
        self.save_timer = node.create_timer(self.save_interval_s, self.add_data)
        self._node.get_logger().info(f"Timer created")

    def reset_callback(self, msg):
        self._reset = True
    
    def timeout_callback(self, msg):
        self._timeout_received = True

    def robot_state_callback(self, msg):
        self._state_msg = msg.pose

    def robot_state_odom_callback(self, msg):
        self._state_msg = msg.pose.pose
        self._twist_msg = msg.twist.twist

    def collision_callback(self, msg):
        margin = 0.0 # Margin is in collision checker now
        if msg.data > margin and self._collision_msg is not None and self._collision_msg.data <= margin:
            self._collisions += 1
            self._node.get_logger().info(f"Collision detected")
        self._collision_msg = msg

    def obstacle_callback(self, msg):
        self._obs_msg = msg

    def weights_callback(self, msg):
        self._weights_msg = msg

    def add_data(self):
        with self._save_lock:

            if self._state_msg and self._obs_msg:
                self._data.new_iteration()

                self._data.add("pos", [self._state_msg.position.x, self._state_msg.position.y])
                self._data.add("v", self._twist_msg.linear.x)
                self._data.add("orientation", self._state_msg.orientation.z)

                for j, obs in enumerate(self._obs_msg.obstacles):
                    idx = obs.id
                    if not self._data.has_data(f"obstacle_{idx}_pos"):
                        self._data.register(f"obstacle_{idx}_pos")
                        self._data.register(f"obstacle_{idx}_orientation")
                        if idx >= self._num_obstacles:
                            self._num_obstacles = idx + 1

                obstacles_received = [False] * self._num_obstacles
                for j, obs in enumerate(self._obs_msg.obstacles):
                    idx = obs.id
                    self._data.add(f"obstacle_{idx}_pos", [obs.pose.position.x, obs.pose.position.y])
                    self._data.add(f"obstacle_{idx}_orientation", quaternion_to_yaw(obs.pose.orientation))
                    obstacles_received[idx] = True

                for idx, o in enumerate(obstacles_received):
                    if not o:
                        self._data.add(f"obstacle_{idx}_pos", [-1e9, -1e9])
                        self._data.add(f"obstacle_{idx}_orientation", -1e9)

            if self._weights_msg:
                weights = dict()
                for weight in self._weights_msg.weights:
                    weights[weight.name] = weight.value
                weights["iteration"] = self._data.get_iteration()

                self._weights_msg = None  # Only save when received

                self._data.add("weights", weights)

            if self._reset:
                self._reset = False

                self._data.add_metric("timeout", copy.deepcopy(int(self._timeout_received)))
                self._timeout_received = False

                # Add metrics
                self._data.add_metric("collisions", copy.deepcopy(self._collisions))
                self._collisions = 0

                self._data.end_experiment()

    def save_data(self):
        with self._save_lock:
            if self._data_folder == '':
                base_data_folder = pathlib.Path(__file__).parent.resolve() / "../data"
            else:
                base_data_folder = self._data_folder
            data_path = f"{base_data_folder}/{self.scenario}"

            os.makedirs(f'{data_path}', exist_ok=True)
            filepath = f'{data_path}/{self.experiment}.json'

            # Save the data to a JSON file
            with open(filepath, 'w') as f:
                json.dump(self._data.get_data(), f, indent=4)

            self._node.get_logger().info(f'Data saved to {filepath}')


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
