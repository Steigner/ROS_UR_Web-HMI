# !! UNCOMENT HERE !!
#!/usr/bin/env python3
# ROS Noetic and up

# #!/usr/bin/env python2
# ROS Melodic and down

# python library -> Subprocess management
import subprocess

# python library -> Set handlers for asynchronous events
import signal

# python library -> Time access and conversions
import time

# ROS python library -> pure python client library for ROS
import rospy

# ROS python library -> common message types
from std_msgs.msg import String

class Switch(object):
    def __init__(self):
        self.robot = None
    
    def con_rob(self):
        """
        Run python script robot control as subprocess.

        :param: None
        :returns: None
        """
        self.start_R4 = subprocess.Popen(["rosrun", "ur_web_control", "control_robot.py"])

    def con_rob_kill(self):
        """
        Quit runned python script robot control.

        :param: None
        :returns: None
        """
        self.start_R4.send_signal(signal.SIGINT)

    def connect(self, robot, ip):
        """
        Run ROS launch file from Universal_Robots_ROS_Driver as subprocess. Save robot info for run MoveIT launch file.
        https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
        
        :param robot, ip: robot type -> URXe, ip adress of robot
        :returns: None
        """
        self.robot = robot
        self.start_R_1 = subprocess.Popen(["roslaunch", "ur_robot_driver", robot.lower() + "_bringup.launch", "robot_ip:=" + ip])
    
    def moveit_run(self):
        """
        Run ROS Industrial MoveIT launch file from Universal_Robots as subprocess.
        https://github.com/ros-industrial/universal_robot
        
        :param: None
        :returns: None
        """
        self.start_R_2 = subprocess.Popen(["roslaunch", self.robot.lower() + "_moveit_config", "moveit_planning_execution.launch"])

    def disconnect(self):
        """
        Quit runned python ROS UR Driver + MoveIT as subprocess.

        :param: None
        :returns: None
        """
        self.start_R_1.send_signal(signal.SIGINT)
        time.sleep(2)
        self.start_R_2.send_signal(signal.SIGINT)
    
        
class SwitchNode(object):
    def __init__(self):
        """
        Create ros node and subscriber with callback.

        :param: None
        :returns: None
        """
        self.switch = Switch()

        rospy.init_node('SWITCH', anonymous=True)
        
        self.subscriber = rospy.Subscriber('/switch', String, self.callback, queue_size=10)

        rospy.spin()
        
    def callback(self, data):
        """
        Switch callback for method in Swich class.

        :param: data -> string
        :returns: None
        """
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