#!/usr/bin/env python

import rospy
import json
import os
import pathlib
import copy

from geometry_msgs.msg import PoseStamped
from mpc_planner_msgs.msg import ObstacleArray, WeightArray  # Replace `your_package` with your actual package name
from std_msgs.msg import String, Empty, Float64

import tf.transformations
from geometry_msgs.msg import Quaternion

import threading

def quaternion_to_yaw(quaternion):
    # Convert the quaternion to a list [x, y, z, w]
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    
    # Convert the quaternion to roll, pitch, and yaw
    _, _, yaw = tf.transformations.euler_from_quaternion(q)
    
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
    
    # Metrics have a different size
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


class DataRecorderNode:
    def __init__(self):
        # Initialize node
        # rospy.init_node('data_recorder_node')

        # Get the scenario name from the command line argument
        self.scenario = rospy.get_param('scenario', 'none')
        self.experiment = rospy.get_param('experiment', 'none')
        self._data_folder = rospy.get_param('data_folder', '')

        self.save_interval_s = 50. / 1000. # Every 50 ms
        self.num_experiments = rospy.get_param('/condition_check/num_experiments', 1)

        # Initialize storage for the topics
        self._data = RecordedData(self.save_interval_s)

        self._data.add_singular_data("scenario", self.scenario)
        self._data.add_singular_data("experiment", self.experiment)

        self._reset = False

        self._state_msg = None
        self._data.register("pos")
        self._data.register("v")
        self._data.register("orientation")

        self._collision_msg = None
        self._collisions = 0
        self._data.register_metric("collisions")

        self._timeout_received = False
        self._data.register_metric("timeout")

        self._obs_msg = None # Note: registered at first occurence
        self._num_obstacles = 0

        self._weights_msg = None
        self._data.register("weights")

        # Subscribers
        rospy.Subscriber('/jackal_socialsim/reset', Empty, self.reset_callback) # End of experiment
        rospy.Subscriber('/condition_check/timeout', Empty, self.timeout_callback) # End of experiment

        rospy.Subscriber('robot_state', PoseStamped, self.robot_state_callback)
        rospy.Subscriber('pedestrian_simulator/collision_detected', Float64, self.collision_callback)
        rospy.Subscriber('/pedestrian_simulator/trajectory_predictions', ObstacleArray, self.obstacle_callback)
        rospy.Subscriber('hey_robot/weights', WeightArray, self.weights_callback)

        # Timer for saving data
        self._save_lock = threading.Lock()
        self.save_timer = rospy.Timer(rospy.Duration(self.save_interval_s), self.add_data)

    def reset_callback(self, msg):
        self._reset = True
    
    def timeout_callback(self, msg):
        self._timeout_received = True

    def robot_state_callback(self, msg):
        self._state_msg = msg

    def collision_callback(self, msg):
        if msg.data > 0. and self._collision_msg is not None and self._collision_msg.data <= 0.:
            self._collisions += 1
        self._collision_msg = msg

    def obstacle_callback(self, msg):
        self._obs_msg = msg

    def weights_callback(self, msg):
        self._weights_msg = msg

    def add_data(self, event):
        with self._save_lock:

            if self._state_msg and self._obs_msg:
                self._data.new_iteration()

                self._data.add("pos", [self._state_msg.pose.position.x, self._state_msg.pose.position.y])
                self._data.add("v", self._state_msg.pose.position.z)
                self._data.add("orientation", self._state_msg.pose.orientation.z)

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
                    # prediction = obs.gaussians[0].mean.poses[0]
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

                self._weights_msg = None # Only save when received

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

            rospy.loginfo(f'Data saved to {filepath}')

# if __name__ == '__main__':
#     try:
#         node = DataRecorderNode()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass
