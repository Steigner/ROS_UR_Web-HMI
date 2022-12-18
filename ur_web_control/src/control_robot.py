#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

from sensor_msgs.msg import JointState
from math import radians

class Robot_Control(object):
    def __init__(self):    
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('TESTOS', anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        
    def go_to_joint(self, data):
        joint_goal = self.move_group.get_current_joint_values()

        if "#base" in data.name:
            joint_goal[0] = radians(data.position[0])

        if "#shoulder" in data.name:
            joint_goal[1] = radians(data.position[1])
            
        if "#elbow" in data.name:
            joint_goal[2] = radians(data.position[2])
        
        if "#wrist_1" in data.name:
            joint_goal[3] = radians(data.position[3])

        if "#wrist_2" in data.name:
            joint_goal[4] = radians(data.position[4])

        if "#wrist_3" in data.name:
            joint_goal[5] = radians(data.position[5])
        
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def listener(self):
        rospy.Subscriber("action_joint_data", JointState , self.go_to_joint)
        rospy.spin()
    
def control_robot():
    robot = Robot_Control()

    while not rospy.is_shutdown():
        robot.listener()

if __name__ == "__main__":
    control_robot()