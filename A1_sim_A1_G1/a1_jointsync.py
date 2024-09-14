      
#!/usr/bin/env python3
# @copyright Galaxea

import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState


class A1_wrapper():
    def __init__(self):
        rospy.init_node('position_command_syncer')
        self.pub = rospy.Publisher('/isaac_sim/joint_command', JointState, queue_size=10)
        self.sub = rospy.Subscriber('joint_states', JointState, self.rviz_callback)   
    def rviz_callback(self, msg):
    	# print("received message")
        msg_copy = JointState()
        print(msg)
        print("received message")
        #if len(msg_copy.position) != 8:
        msg_copy.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'arm_joint6', 'gripper_axis1', 'gripper_axis2']
        msg_copy.position = [i for i in msg.position[:6]] + [0.05, 0.05]
        msg_copy.velocity = [] #if msg_copy.velocity == None else [i for i in msg_copy.velocity] + [0, 0]
        self.pub.publish(msg_copy)

if __name__ == '__main__':
    A1_wrapper()
    rospy.spin()
    

    
