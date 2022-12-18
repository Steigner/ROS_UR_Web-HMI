#!/usr/bin/env python3

import subprocess
import signal
import rospy
import time
from std_msgs.msg import String

class Switch(object):
    def __init__(self):
        self.robot = None
    
    def con_rob(self):
        self.start_R4 = subprocess.Popen(["rosrun", "ur_web_control", "control_robot.py"])

    def con_rob_kill(self):
        self.start_R4.send_signal(signal.SIGINT)

    def connect(self, robot, ip):  
        self.robot = robot
        self.start_R_1 = subprocess.Popen(["roslaunch", "ur_robot_driver", robot.lower() + "_bringup.launch", "robot_ip:=" + ip])
        # time.sleep(5)
        # self.start_R_2 = subprocess.Popen(["roslaunch", robot.lower() + "_moveit_config", "moveit_planning_execution.launch"])
    
    def moveit_run(self):
        self.start_R_2 = subprocess.Popen(["roslaunch", self.robot.lower() + "_moveit_config", "moveit_planning_execution.launch"])

    def disconnect(self):
        self.start_R_1.send_signal(signal.SIGINT)
        time.sleep(2)
        self.start_R_2.send_signal(signal.SIGINT)
    
        
class SwitchNode(object):
    def __init__(self):
        self.switch = Switch()

        rospy.init_node('switch', anonymous=True)
        
        self.subscriber = rospy.Subscriber('/switch', String, self.callback, queue_size=10)
        
        print('Created!')
        
        rospy.spin()
        
    def callback(self, data):
        if data.data == 'disconnect':
            self.switch.disconnect()

        elif data.data == 'manual_control':
            self.switch.con_rob()

        elif data.data == 'manual_control_stop':
            self.switch.con_rob_kill()

        elif data.data == 'moveit_run':
            self.switch.moveit_run()

        else:
            robot_ip = data.data.split("-")
            self.switch.connect(robot_ip[0], robot_ip[1])

if __name__ == '__main__':
    while not rospy.is_shutdown():
        node = SwitchNode()