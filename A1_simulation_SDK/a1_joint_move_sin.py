      
#!/usr/bin/env python3
# @copyright Galaxea

import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState

rospy.init_node('position_command_publisher')
pub = rospy.Publisher('/isaac_sim/joint_command', JointState, queue_size=10)

joint_state_position = JointState()

joint_state_position.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'arm_joint6']
## arbitary joint value threshold
max_joint_value = np.array([30, 40.0, 0,    30.0,  30.0,  165.0]) / 180.0 * np.pi
min_joint_value = np.array([-30, 0.0,  -90,    -30.0,  -30.0,  165.0]) / 180.0 * np.pi

rate = rospy.Rate(10)
start_time = time.time()
while not rospy.is_shutdown():
    current_time = time.time()
    phase = (current_time - start_time) / 10.0 * 2 * np.pi
    joint_position = min_joint_value+ 0.8*(max_joint_value - min_joint_value) + 0.8 * (max_joint_value - min_joint_value) * np.sin(phase) 
    print(joint_position)
    joint_state_position.position = joint_position.tolist()
    pub.publish(joint_state_position)
    rate.sleep()