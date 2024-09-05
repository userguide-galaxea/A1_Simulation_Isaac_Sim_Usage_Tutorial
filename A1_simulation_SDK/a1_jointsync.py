      
#!/usr/bin/env python3
# @copyright Galaxea

import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState


class A1_wrapper():
    def __init__(self):
        rospy.init_node('position_command_publisher')
        self.pub = rospy.Publisher('/isaac_sim/joint_command', JointState, queue_size=10)
        self.sub = rospy.Subscriber('/joint_states', JointState, self.rviz_callback)   
    def rviz_callback(self, msg):
        msg_sync = JointState()
        msg_sync.header = msg.header
        msg_sync.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'arm_joint6']
        msg_sync.position = [i for i in msg.position] 
        self.pub.publish(msg_sync)
if __name__ == '__main__':
    A1_wrapper()
    rospy.spin()
    

    
