$(function init() {
    var ros = new ROSLIB.Ros();

    ros.connect('ws://' + Cookies.get('ros_ip_adress') + ':9090');

    var viewer = new ROS3D.Viewer({
        divID: 'urdf',
        width: $('#left.column').width() - 50,
        height: $('#left.column').height() - 50,
        background: '#F2F2F2',
        intensity: 1.6,
        antialias: true,
        cameraPose: { x: 1, y: 1, z: 1 },
    });

    var tfClient = new ROSLIB.TFClient({
        ros: ros,
        //fixedFrame : 'world',
        angularThres: 0.001,
        transThres: 0.001,
        rate: 144,
    });

    var urdfClient = new ROS3D.UrdfClient({
        ros: ros,
        tfClient: tfClient,
        path: './static',
        rootObject: viewer.scene,
    });



// -----------------------------------------------------------------------------------------------

class RobotControl {
    constructor(ros) {
        this.ros = ros;
        this.timeOut = 0;
        this.count = 0;
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
    // Also fast calculator from quaternion to euler.
    data_feed() {
        var cartesian_data = this.cartesian_data;

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
            function quaternion_to_euler(q0, q1, q2, q3) {
                var x_rx = 1 - 2 * (q1 ** 2 + q2 ** 2);
                var y_rx = 2 * (q0 * q1 + q2 * q3);
                var RX = Math.atan2(y_rx, x_rx);

                var x_ry = 2 * (q0 * q2 - q3 * q1);
                var RY = Math.asin(x_ry);

                var y_rz = 2 * (q0 * q3 + q1 * q2);
                var x_rz = 1 - 2 * (q2 ** 2 + q3 ** 2);
                var RZ = Math.atan2(x_rz, y_rz);

                // euler position
                $('#RX').text(Math.round(RZ * (180 / Math.PI)) + ' °');
                $('#RY').text(Math.round(RY * (180 / Math.PI)) + ' °');
                $('#RZ').text(Math.round(RX * (180 / Math.PI)) + ' °');
            }

            quaternion_to_euler(q0, q1, q2, q3);
        });

        var get_data_joint = this.get_data_joint;

        // get joint position of each motor
        get_data_joint.subscribe(function (message) {
            var elbow = message.position[0] * (180 / Math.PI);
            var shoulder = message.position[1] * (180 / Math.PI);
            var base = message.position[2] * (180 / Math.PI);
            var wrist_1 = message.position[3] * (180 / Math.PI);
            var wrist_2 = message.position[4] * (180 / Math.PI);
            var wrist_3 = message.position[5] * (180 / Math.PI);
            
            $('#base.form-range').val(base).change();
            $('#shoulder.form-range').val(shoulder).change();
            $('#elbow.form-range').val(elbow).change();
            $('#wrist_1.form-range').val(wrist_1).change();
            $('#wrist_2.form-range').val(wrist_2).change();
            $('#wrist_3.form-range').val(wrist_3).change();
            
            $('#base.joint-data').text(Math.round(base) + ' °');
            $('#shoulder.joint-data').text(Math.round(shoulder) + ' °');
            $('#elbow.joint-data').text(Math.round(elbow) + ' °');
            $('#wrist_1.joint-data').text(Math.round(wrist_1) + ' °');
            $('#wrist_2.joint-data').text(Math.round(wrist_2) + ' °');
            $('#wrist_3.joint-data').text(Math.round(wrist_3) + ' °');

        });
    }

    robot_status() {
        var status_topic = this.status_topic;

        status_topic.subscribe(function (message) {
            var status = message.status.status;
            console.log(status)
            if (status == 3 || status == 4) {
                    // if status 4 -> error!!
                    $(':button, :input').prop('disabled', false); // Enable all the buttons
                    //$(':input').prop('disabled', false); // Enable all the inputs
                    console.log("Done")
            } else {
                return;
            }
        });
    }

    on_clicker(elements){
        var joint_act = this.joint_act;
        var get_data_joint = this.get_data_joint;

        $("#btn_run").click(function() {
            $(':button, :input').prop('disabled', true); // Disable all the buttons
            //$(':input').prop('disabled', true); // Disable all the buttons

            var q = [];
            for (var element of elements) {
                q.push(parseInt($(element + '.form-range').val()));
            }

            var joint = new ROSLIB.Message({
                name: elements,
                position: q,
            });

            joint_act.publish(joint);

            get_data_joint.subscribe(function (message) {
                var elbow = message.position[0] * (180 / Math.PI);
                var shoulder = message.position[1] * (180 / Math.PI);
                var base = message.position[2] * (180 / Math.PI);
                var wrist_1 = message.position[3] * (180 / Math.PI);
                var wrist_2 = message.position[4] * (180 / Math.PI);
                var wrist_3 = message.position[5] * (180 / Math.PI);
                
                $('#base.form-range').val(base).change();
                $('#shoulder.form-range').val(shoulder).change();
                $('#elbow.form-range').val(elbow).change();
                $('#wrist_1.form-range').val(wrist_1).change();
                $('#wrist_2.form-range').val(wrist_2).change();
                $('#wrist_3.form-range').val(wrist_3).change();
                
                $('#base.joint-data').text(Math.round(base) + ' °');
                $('#shoulder.joint-data').text(Math.round(shoulder) + ' °');
                $('#elbow.joint-data').text(Math.round(elbow) + ' °');
                $('#wrist_1.joint-data').text(Math.round(wrist_1) + ' °');
                $('#wrist_2.joint-data').text(Math.round(wrist_2) + ' °');
                $('#wrist_3.joint-data').text(Math.round(wrist_3) + ' °');
            });
        });
    }

    // public method:
    //   input: none
    //   return none
    // Note: there we checking status from move group, if it done or in proceses etc ...
    on_slider(slider) {
        var get_data_joint = this.get_data_joint;

        $(document).on('input', slider, function() {
            get_data_joint.unsubscribe();
            var value = $(slider + '.form-range').val();
            $(slider + '.joint-data').text(value + ' °');
        });
    }

    // public method:
    //   input: defined left button
    //   return none
    // Note: if is touch on button stop subscribing and start move to left,
    // then start subscribe again
    on_left_right(element) {
        var get_data_joint = this.get_data_joint;

        $(element+ '.left').on('touchend mousedown', function () {
            get_data_joint.unsubscribe();
            var value = parseInt($(element + '.form-range').val());
            value -= 5
            $(element + '.form-range').val(value)
            $(element + '.joint-data').text(value + ' °');
        });

        $(element+ '.right').on('touchend mousedown', function () {
            get_data_joint.unsubscribe();
            var value = parseInt($(element + '.form-range').val());
            value += 5
            $(element + '.form-range').val(value)
            $(element + '.joint-data').text(value + ' °');
        });
    }

    kill_process() {
        Cookies.remove('ros_ip_adress');

        console.log('killiung');
        var data = new ROSLIB.Message({
            data: 'manual_control_stop',
        });
        var data2 = new ROSLIB.Message({
            data: 'disconnect',
        });
        this.switch.publish(data);
        this.switch.publish(data2);

        window.location.href = '/';
    }

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
        var elements = [
            '#base',
            '#shoulder',
            '#elbow',
            '#wrist_1',
            '#wrist_2',
            '#wrist_3',
        ];
        
        this.on_clicker(elements);
        for (var element of elements) {
            this.on_slider(element);
            this.on_left_right(element);
        }

        this.robot_status();  
    }
}

let robot = new RobotControl(ros);

robot.main();

$("#btn_home").click(function() {
    robot.kill_process()
})

// prevent to open dialog windwo of touch
window.oncontextmenu = function (event) {
    event.preventDefault();
    event.stopPropagation();
    return false;
};

});

window.onbeforeunload = function (e) {
    e = e || window.event;

    // For IE and Firefox prior to version 4
    if (e) {
        e.returnValue = 'Sure?';
    }

    // For Safari
    return 'Sure?';
};