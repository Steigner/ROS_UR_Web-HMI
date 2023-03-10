## ROS (Robot Operating System)

1) First of all you need to have [ROS](http://wiki.ros.org/ROS/Installation) + [MoveIT](https://moveit.ros.org/install/) installed.
2) Imply this meta-package into catkin workspace with other depandacies -> create according to the tutorial [Universal Robots ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#building).
3) Install [Rosbridge](http://wiki.ros.org/rosbridge_suite)
```console
user@user-pc:~$ sudo apt-get install ros-<rosdistro>-rosbridge-server
```
4) Install [TF2-Web-Republisher](http://wiki.ros.org/tf2_web_republisher)
```console
user@user-pc:~$ sudo apt-get install ros-<rosdistro>-tf2-web-republisher
```
5) To the catkin workspace, by default *catkin_ws* insert the package **ur_web_control**.
6) You can then start the system by the command:

```console
$ roslaunch ur_web_control ur_web_constrol.launch
```
