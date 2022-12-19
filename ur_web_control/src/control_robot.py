# #!/usr/bin/env python3
# ROS Noetic and up

# #!/usr/bin/env python2
# ROS Melodic and down

# python library -> System-specific parameters and functions
import sys

# ROS python library -> pure python client library for ROS
import rospy

# python library -> math import radians function
from math import radians

# ROS Moveit python library -> python interfaces to MoveIt
import moveit_commander

# ROS python library -> messages for commonly used sensors
from sensor_msgs.msg import JointState

class Robot_Control(object):
    def __init__(self):
        """
        Inicialize moveit system + node

        :param: None
        :returns: None
        """
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('TESTOS', anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        
    def go_to_joint(self, data):
        """
        Callback on input data change joints rotation.

        :param: data -> sensor_msgs.msg.JointState
        :returns: None
        """
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
        """
        Subscribe data on topic with JointState mess type with
        go_to_joint() method.

        :param: None
        :returns: None
        """
        rospy.Subscriber("action_joint_data", JointState , self.go_to_joint)
        rospy.spin()

# It was generally more comfortable to run whole this script from switch.py, than 
# call only method.
def control_robot():
    robot = Robot_Control()

    while not rospy.is_shutdown():
        robot.listener()

if __name__ == "__main__":
    control_robot()