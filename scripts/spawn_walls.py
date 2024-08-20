#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion

def spawn_wall():
    # Initialize ROS node
    rospy.init_node('spawn_wall_node')

    # Wait for the service to be available
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    # Load the SDF string for the wall
    sdf = """
    <sdf version="1.6">
      <model name="wall">
        <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>10 0.1 2.5</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>10 0.1 2.5</size>
              </box>
            </geometry>
            <material>
                <ambient>0.0 0.0 0.0 1.0</ambient>   <!-- Ambient color: black -->
                <diffuse>0.0 0.0 0.0 1.0</diffuse>   <!-- Diffuse color: black -->
                <specular>0.1 0.1 0.1 1.0</specular> <!-- Specular color: dark grey -->
                <emissive>0.0 0.0 0.0 1.0</emissive> <!-- Emissive color: black -->
            </material>
          </visual>
          <pose>0 0 1.25 0 0 0</pose>
        </link>
      </model>
    </sdf>
    """
    
    try:
        # Create a handle to the service
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        # Define the initial pose of the wall
        initial_pose = Pose()
        initial_pose.position.x = 5
        initial_pose.position.y = 5
        initial_pose.position.z = 0
        initial_pose.orientation = Quaternion(0, 0, 0, 1)

        # Call the service to spawn the wall
        spawn_model_prox("wall", sdf, "", initial_pose, "world")
        rospy.loginfo("Wall spawned successfully!")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        spawn_wall()
    except rospy.ROSInterruptException:
        pass
