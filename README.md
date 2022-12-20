## ROS Universal Robots Web Human-Machnie Interface

Within this project, a web-based HMI (Human-Machine Interface) was created. The basic function of this work is to easily control individual joints of collaborative robots from the Universal Robots stable using mainly ROS (Robot Operating System). The system also enables web browser visualization of Rviz. **The project offers a high degree of modularity and offer to you make your own modifications by your preferences**

 :arrow_right: **It can be easily extended to other robots from other manufacturers.**

```javascript
Software
------------------------------------
|
+ Backend
+ ---------
| ROS
|   - version Noetic, Melodic (Tested)
| FLASK
|   - version 2.2.2
|
+ Frontend
+ ---------
| JAVASCRIPT
|   - roslibjs version 1.1.0
|       - ros3djs + dependencies
|   - jQuery version 3.6.2
|   - bootstrap version 5.0.2
|   - js-cookie version 3.0.1
| CSS
|   - bootstrap version 5.0.2
| HTML
```

The system is designed so that especially the basic control logic is implemented on the **client side**, so the system can be easily adapted to any existing web framework / server.

**If anyone is more conservative, he can override the system with more server-side logic. However, this choice comes at the price of less modularity.**

![Structure](/docs/structure.png)

### Note 
Switch logic on client side in *robot_connect.js* relies on logs /rosout statements (URx bringup, Moveit).

The solution was mainly based on rostopic due to simplicity than rosservices. Rosservices would mainly be used in real deployment.

🔴 Server e.g. uWSGI, Gunicorn

🔵 URx collaborative robot

🟢 ROS Server e.g. Nvidia Xavier, Raspberry Pi

## :information_source: Contacts

:mailbox: m.juricek@outlook.com