#!/usr/bin/env python3
# @copyright Galaxea
import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState

def main():
    # Initialize the ROS node
    rospy.init_node('trajectory_command_publisher')
    pub = rospy.Publisher('/isaac_sim/joint_command', JointState, queue_size=10)

    # Load the trajectory from the .npz file
    data = np.load('joint_trajectory.npz')
    joint_names = data['name'].tolist()
    trajectory = data['position']

    joint_state_position = JointState()
    joint_state_position.name = joint_names

    rate = rospy.Rate(10)  # 10 Hz
    for joint_position in trajectory:
        if rospy.is_shutdown():
            break

        joint_state_position.position = joint_position.tolist()
        pub.publish(joint_state_position)

        print(joint_position)  # Optional: print the current joint position
        rate.sleep()

if __name__ == '__main__':
    main()
