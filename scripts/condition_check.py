#!/usr/bin/env python
import time
import math

import tf
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

class RobotStateMonitor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('condition_check')

        # Get the parameters for the conditions
        self.condition_type = rospy.get_param('~condition_type', 'close_to_point')  # 'close_to_point' or 'exceed_value'

        self.target_x = rospy.get_param('~target_x', 0.0)
        self.target_y = rospy.get_param('~target_y', 0.0)
        self.threshold_distance = rospy.get_param('~threshold_distance', 1.0)

        self.max_x = rospy.get_param('~max_x', 10.0)
        self.max_y = rospy.get_param('~max_y', 10.0)

        self.timeout = rospy.get_param('~timeout', 10.0)
        self.start_time = time.perf_counter()

        self.num_experiments = rospy.get_param('~num_experiments', 1)

        # Publisher for the reset topic
        self.reset_pub = rospy.Publisher('/jackal_socialsim/reset', Empty, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.timeout_pub = rospy.Publisher('/condition_check/timeout', Empty, queue_size=5)

        # Subscriber for the robot_state topic
        self.state_sub = rospy.Subscriber('robot_state', PoseStamped, self.state_callback)

        self.completions = 0
        self.timeouts = 0
        
        self.first = True


    def state_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        should_reset = False
        elapsed_time = time.perf_counter() - self.start_time

        if self.is_robot_flipped(msg) and elapsed_time > 1:
            self.start_time = time.perf_counter() # Prevent double resets
            self.publish_reset()
            return

        if self.first:
            self.first = False
            self.publish_reset() # Reset but without counting this one
            return


        if self.condition_type == 'close_to_point':
            # Condition 1: Check if the robot is close to a predefined point
            distance = ((x - self.target_x) ** 2 + (y - self.target_y) ** 2) ** 0.5
            if distance <= self.threshold_distance:
                should_reset = True

        elif self.condition_type == 'exceed_value':
            # Condition 2: Check if both x and y exceed certain values
            if x >= self.max_x or y >= self.max_y:
                should_reset = True
        
        if not should_reset:
            if elapsed_time > self.timeout:
                print("Experiment: Timeout")
                should_reset = True
                self.timeouts += 1
                self.publish_timeout()

        if should_reset and elapsed_time > 1.:
            self.start_time = time.perf_counter() # Prevent double resets

            self.publish_reset()
            if self.completions == self.num_experiments:
                rospy.sleep(5.)
                print(f"{self.num_experiments} experiments completed!")
                rospy.signal_shutdown("Experiments completed")
            else:
                self.completions += 1
                print(f"Experiment {self.completions}/{self.num_experiments}")


    def publish_reset(self):
        # Publish an empty message to reset the simulation
        # rospy.loginfo_throttle(1, "Condition met. Publishing reset.")
        self.reset_pub.publish(Empty())
        self.publish_goal()
        rate = rospy.Rate(5)
        for i in range(10):
            rate.sleep()
            self.publish_goal()

        self.start_time = time.perf_counter()

    def publish_timeout(self):
        self.timeout_pub.publish(Empty())

    def publish_goal(self):
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = self.target_x
        goal_msg.pose.position.y = self.target_y
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        self.goal_pub.publish(goal_msg)


    def is_robot_flipped(self, pose_stamped):
        # Define the threshold (1/8 * pi)
        threshold = math.pi / 8
        
        # Check if the pitch or roll is greater than the threshold
        return abs(pose_stamped.pose.orientation.x) > threshold or abs(pose_stamped.pose.orientation.y) > threshold

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    monitor = RobotStateMonitor()
    monitor.run()
