$(function init() {
    class RobotControl {
        constructor(ros) {
            this.ros = ros;
            this.switch = null;
            this.cartesian_data = null;
            this.get_data_joint = null;
            this.joint_act = null;
            this.status_topic = null;
        }

        // public method:
        //   input: none
        //   return none
        // Note: In this method we defined topics and parametres for communication
        // with ROS websocket server, which control robot.
        rosnode_init() {
            // ROS NODES
            // emergency stop node
            this.switch = new ROSLIB.Topic({
                ros: this.ros,
                name: '/switch',
                messageType: 'std_msgs/String',
            });

            // init current position node
            this.cartesian_data = new ROSLIB.Topic({
                ros: this.ros,
                name: '/tf',
                messageType: 'tf2_msgs/TFMessage',
            });

            // init joint states node
            this.get_data_joint = new ROSLIB.Topic({
                ros: this.ros,
                name: '/joint_states',
                messageType: 'sensor_msgs/JointState',
            });

            // init joint action data
            this.joint_act = new ROSLIB.Topic({
                ros: this.ros,
                name: '/action_joint_data',
                messageType: 'sensor_msgs/JointState',
            });

            // init feedback from moveit
            this.status_topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/move_group/feedback',
                messageType: 'moveit_msgs/MoveGroupActionFeedback',
            });
        }

        // public method:
        //   input: none
        //   return none
        // Note: Feeding data to labels about actual TCP position.
        // Also fast calculator from quaternion to euler angles.
        data_feed() {
            var cartesian_data = this.cartesian_data;
            var get_data_joint = this.get_data_joint;
            cartesian_data.subscribe(function (message) {
                //console.log(message.transforms[0].transform.rotation.x)

                // cartesian position
                $('#X').text(Math.round(message.transforms[0].transform.translation.x * 100) + ' cm');
                $('#Y').text(Math.round(message.transforms[0].transform.translation.y * 100) + ' cm');
                $('#Z').text(Math.round(message.transforms[0].transform.translation.z * 100) + ' cm');

                var q0 = message.transforms[0].transform.rotation.x;
                var q1 = message.transforms[0].transform.rotation.y;
                var q2 = message.transforms[0].transform.rotation.z;
                var q3 = message.transforms[0].transform.rotation.w;

                // method currying -> function inside method
                // wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
                function quaternion_to_euler(qx, qy, qz, qw) {
                    var sinr = 2 * (qw * qx + qy * qz);
                    var cosr = 1 - 2 * (qx ** 2 + qy ** 2);
                    var RX = Math.atan2(sinr, cosr);

                    var sinp = Math.sqrt(1 + 2 * (qw * qx - qy * qz));
                    var cosp = Math.sqrt(1 - 2 * (qw * qx - qy * qz));
                    var RY = 2 * Math.atan2(sinp, cosp) 

                    var siny = 2 * (qw * qz + qx * qy);
                    var cosy = 1 - 2 * (qy ** 2 + qz ** 2);
                    var RZ = Math.atan2(siny, cosy);

                    // euler position
                    $('#RX').text(Math.round(RZ * (180 / Math.PI)) + ' °');
                    $('#RY').text(Math.round(RY * (180 / Math.PI)) + ' °');
                    $('#RZ').text(Math.round(RX * (180 / Math.PI)) + ' °');
                }
                // compute euler angles from quaternion
                quaternion_to_euler(q0, q1, q2, q3);
            });

            // get joint position of each motor
            get_data_joint.subscribe(function (message) {
                // transfer data to degrees
                var elbow = message.position[0] * (180 / Math.PI);
                var shoulder = message.position[1] * (180 / Math.PI);
                var base = message.position[2] * (180 / Math.PI);
                var wrist_1 = message.position[3] * (180 / Math.PI);
                var wrist_2 = message.position[4] * (180 / Math.PI);
                var wrist_3 = message.position[5] * (180 / Math.PI);
                
                // data to slider input
                $('#base.form-range').val(base).change();
                $('#shoulder.form-range').val(shoulder).change();
                $('#elbow.form-range').val(elbow).change();
                $('#wrist_1.form-range').val(wrist_1).change();
                $('#wrist_2.form-range').val(wrist_2).change();
                $('#wrist_3.form-range').val(wrist_3).change();
                
                // data to label
                $('#base.joint-data').text(Math.round(base) + ' °');
                $('#shoulder.joint-data').text(Math.round(shoulder) + ' °');
                $('#elbow.joint-data').text(Math.round(elbow) + ' °');
                $('#wrist_1.joint-data').text(Math.round(wrist_1) + ' °');
                $('#wrist_2.joint-data').text(Math.round(wrist_2) + ' °');
                $('#wrist_3.joint-data').text(Math.round(wrist_3) + ' °');
            });
        }

        // public method:
        //   input: none
        //   return none
        // Note: Check status from ROS sever, based on status enable inputs
        robot_status() {
            var status_topic = this.status_topic;
            status_topic.subscribe(function (message) {
                var status = message.status.status;
                if (status == 3 || status == 4) {
                        // if status 4 -> error!!
                        // Enable all the buttons, inputs
                        $(':button, :input').prop('disabled', false); 
                        console.log("Done")
                } else {
                    return;
                }
            });
        }

        // public method:
        //   input: none
        //   return none
        // Note: On click button publish new joint values to ROS Server
        on_run_btnclick(elements){
            var joint_act = this.joint_act;
            var get_data_joint = this.get_data_joint;
            $("#btn_run").click(function() {
                // Disable all the buttons, inputs
                $(':button, :input').prop('disabled', true); 
                
                // empty array
                var q = [];
                for (var element of elements) {
                    // append - parsed data from string to int - slider input
                    q.push(parseInt($(element + '.form-range').val()));
                }
                
                // create Message with joint values array!
                // name: array of string
                // position: array of int
                var joint = new ROSLIB.Message({
                    name: elements,
                    position: q,
                });
                
                // publish data
                joint_act.publish(joint);
                // subscribe data from robot again
                get_data_joint.subscribe();
            });
        }

        // public method:
        //   input: none
        //   return none
        // Note: on touch slider input, stop subscribe data from robot and set 
        // value based on input
        on_slider(slider) {
            var get_data_joint = this.get_data_joint;
            $(document).on('input', slider, function() {
                //on click unsubscribe
                get_data_joint.unsubscribe();
                // get data from slider input
                var value = $(slider + '.form-range').val();
                // change data in slider label
                $(slider + '.joint-data').text(value + ' °');
            });
        }

        // public method:
        //   input: defined left button
        //   return none
        // Note: if is touch on button stop subscribing dat and start 
        // move to left or left
        on_left_right_btnclick(element) {
            var get_data_joint = this.get_data_joint;
            $(element+ '.left').on('touchend mousedown', function () {
                //on click unsubscribe
                get_data_joint.unsubscribe();
                // parse data from string to int - slider input
                var value = parseInt($(element + '.form-range').val());
                // move 5° in - direction
                value -= 5
                // change data in slider input and label
                $(element + '.form-range').val(value)
                $(element + '.joint-data').text(value + ' °');
            });

            $(element+ '.right').on('touchend mousedown', function () {
                //on click unsubscribe
                get_data_joint.unsubscribe();
                // parse data from string to int - slider input
                var value = parseInt($(element + '.form-range').val());
                // move 5° in + direction
                value += 5
                // change data in slider input and label
                $(element + '.form-range').val(value)
                $(element + '.joint-data').text(value + ' °');
            });
        }

        // public method:
        //   input: none
        //   return none
        // Note: Publish data for stop ur drive + moveit file
        kill_process() {
            // remove data from cookie named ros_ip_adress
            Cookies.remove('ros_ip_adress');
            console.log('STOP');

            var data = new ROSLIB.Message({
                data: 'manual_control_stop',
            });

            var data2 = new ROSLIB.Message({
                data: 'disconnect',
            });
            
            this.switch.publish(data);
            this.switch.publish(data2);
            // go to init page
            window.location.href = '/';
        }

        // public method:
        //   input: none
        //   return none
        // Note: Publish data for run moveit launch file
        control_robot() {
            var data = new ROSLIB.Message({
                data: 'manual_control',
            });
            this.switch.publish(data);
        }

        // public method:
        //   input: none
        //   return none
        // Note: MAIN
        main() {
            this.rosnode_init();
            this.control_robot();
            this.data_feed();
            
            // get all joints of robot
            // must match range slider + buttons!!
            var elements = [];
            $('.joint').map(function() {
                // this callback function will be called once for each matching element
                elements.push(this.id); 
            });

            this.on_run_btnclick(elements);
            // check on every element on slide / right left click
            for (var element of elements) {
                this.on_slider(element);
                this.on_left_right_btnclick(element);
            }
            this.robot_status();  
        }
    }

    class RobotShow {
        constructor(){
            this.ros = null;
        }
        
        // public method:
        //   input: none
        //   return none
        // Note: connect to ROS Server
        ros_server_connect(){
            this.ros = new ROSLIB.Ros();
            this.ros.connect('ws://' + Cookies.get('ros_ip_adress') + ':9090');
        }

        // public method:
        //   input: none
        //   return none
        // Note: Show Web based Rviz
        rviz_show(){
            var ros = this.ros;
            // setting app canvas div "urdf"
            // size is calculated from width and height alocated div
            var viewer = new ROS3D.Viewer({
                divID: 'urdf',
                width: $('#left.column').width() - 50,
                height: $('#left.column').height() - 50,
                background: '#F2F2F2',
                intensity: 1.6,
                antialias: true,
                cameraPose: { 
                    x: 1, 
                    y: 1, 
                    z: 1 
                },
            });
            
            // tf client for get proper motion with ros moveit
            var tfClient = new ROSLIB.TFClient({
                ros: ros,
                //fixedFrame : 'world',
                angularThres: 0.001,
                transThres: 0.001,
                rate: 144,
            });
            
            // get data from folder where is located meshes ->
            // for your integration it is neccesary have 
            // "copies" meshes what you use with ros moveit 
            // in server side file system.    
            // Make sure that the folder and subfolder mappings match 
            // the folder system in the catkin workspace. In this example, 
            // the simplest use is to put the mapped system from the 
            // ./universal_robots/ur_description/meshes/...
            // package we have in the catkin workspace into the static folder.
            new ROS3D.UrdfClient({
                ros: ros,
                tfClient: tfClient,
                path: './static',
                rootObject: viewer.scene,
            });
        }

        // public method:
        //   input: none
        //   return ros.connection
        // Note: Main
        main(){
            this.ros_server_connect();
            this.rviz_show();
            return this.ros;
        }
    }

    // connect to ROS web server and create rviz
    var ros = new RobotShow().main();

    // control robot based on ros connection
    var robot = new RobotControl(ros);
    robot.main();

    // on click home button kill procceses
    $("#btn_home").click(function() {
        robot.kill_process()
    })
});

// before leave page check if user is ok with leaving
window.onbeforeunload = function (e) {
    e = e || window.event;

    // For IE and Firefox prior to version 4
    if (e) {
        e.returnValue = 'Sure?';
    }

    // For Safari
    return 'Sure?';
};