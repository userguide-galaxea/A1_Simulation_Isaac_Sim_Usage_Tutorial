      
#!/usr/bin/env python3
# @copyright Galaxea

import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


class A1_wrapper():
    def __init__(self):
        rospy.init_node('mpc_picker', anonymous=True)
        self.pub_js = rospy.Publisher('/isaac_sim/joint_command', JointState, queue_size=10)
        self.pub_mpc = rospy.Publisher('/a1_ee_target', PoseStamped, queue_size=10)
        self.command_js = JointState()
        self.init_command_js()
        self.mpc_target = PoseStamped()
        self.sub = rospy.Subscriber('/joint_states', JointState, self.rviz_callback)   
        self.rate_hz = 100
        self.rate = rospy.Rate(self.rate_hz)
        

    def init_command_js(self):
        self.command_js.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'arm_joint6', 'gripper_axis1', 'gripper_axis2']
        self.command_js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.05]
        self.command_js.velocity = []

    def set_mpc_target(self,x=1,y=0,z=0, x1 = 1, y1 = 0, z1 = 0, w1 = 0):
        self.mpc_target.header.frame_id = ""
        self.mpc_target.pose.orientation.x = x1
        self.mpc_target.pose.orientation.y = y1
        self.mpc_target.pose.orientation.z = z1
        self.mpc_target.pose.orientation.w = w1
        self.mpc_target.pose.position.x = x
        self.mpc_target.pose.position.y = y
        self.mpc_target.pose.position.z = z
    
    def pub_mpc_target(self,duration=4):
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.pub_mpc.publish(self.mpc_target)
            self.rate.sleep()

    def rviz_callback(self, msg):
        self.command_js.position[:6] = msg.position[:6]
        self.pub_js.publish(self.command_js)

    def close_gripper(self):
        self.command_js.position[-2] = 0.015
        self.command_js.position[-1] = 0.015
        rospy.sleep(1)

    def open_gripper(self):
        self.command_js.position[-2] = 0.05
        self.command_js.position[-1] = 0.05
        self.pub_js.publish(self.command_js)
        rospy.sleep(1)

    def pub_mpc_command(self, command_js):
        self.pub.publish(command_js)


if __name__ == '__main__':
    controller_a1 = A1_wrapper()
    pick_height = 0.18
    waypoints = [
                # [0.12,0,0.5],
                [0.3,0,0.5],
                [0.3,0,pick_height],
                [0.4,0,0.5],
                [0,-0.4,0.3],
                [0,-0.4,pick_height],
                [0,-0.4,0.3],
                ]
    controller_a1.open_gripper()
    for i,waypoint_i in enumerate(waypoints):
        controller_a1.set_mpc_target(waypoint_i[0],waypoint_i[1],waypoint_i[2])
        print(f"start move to target:{waypoint_i}") 
        duration = 5
        controller_a1.pub_mpc_target(duration)
        if waypoint_i[0] == 0.3 and waypoint_i[1] == 0 and waypoint_i[2] == pick_height:
            controller_a1.close_gripper()
            print("start close gripper")
        if waypoint_i[0] == 0 and waypoint_i[1] == -0.4 and waypoint_i[2] == pick_height:
            controller_a1.open_gripper()
            print("start open gripper")
        
    