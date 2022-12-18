$(function init() {
    // bootstrap create allert div 
    // https://getbootstrap.com/docs/5.2/components/alerts/#examples
    const alertPlaceholder = $('#liveAlertPlaceholder')
    function alert(message, type){
        const wrapper = document.createElement('div')
        wrapper.innerHTML = [
            `<div class="alert alert-${type} alert-dismissible" role="alert">`,
            `   <div>${message}</div>`,
            '</div>'
        ].join('')

        alertPlaceholder.append(wrapper)
    }

    // on click connect disable all buttons and change button to spinner anim
    $(document).ready(function() {
        $("#connect").click(function() {
        // disable button
        $(':button').prop('disabled', true)
        $(this).prop("disabled", true);
        // add spinner to button
        $(this).html(
            '<span class="spinner-grow spinner-grow-sm" role="status" aria-hidden="true"></span>'
        );
        });
    });

    // on click show up modal window
    $(".ur").click(function(event){
        $('#modal-title').text(event.target.id)
        console.log(event.target.id);
    });

    // check jquery input ip adress in correct form ipv4
    $('#ip-adress, #ros-ip-adress').inputmask({
        alias: "ip",
        greedy: false
    });

    // on click connect -> connect to ROS Server
    $('#connect').click(function () {
        var ros = new ROSLIB.Ros();

        // connect to ROS
        ros.connect('ws://' + $('#ros-ip-adress').val() + ':9090');

        // if is succesfull connection set cookie ip adress
        ros.on('connection', function () {
            alert('Succesfull! Connection to Server! Connecting to robot!', 'success')
            Cookies.set('ros_ip_adress', $('#ros-ip-adress').val())
        });
        // on warnign show allert
        ros.on('close', function () {
            alert('Connection to Server closed!', 'warning')
        });
        // on error show alert -> reload page
        ros.on('error', function () {
            alert('Connection to Server failed! Reload page after 5 seconds!', 'danger')
            window.setTimeout(function () {
                window.location.href = '/';
            }, 5000); //5seonds
        });

        // create ROS publish topic name /switch type String
        var pub = new ROSLIB.Topic({
            ros: ros,
            name: '/switch',
            messageType: 'std_msgs/String',
        });

        // create message
        var mess = new ROSLIB.Message({
            data: $('#modal-title').text() + '-' + $('#ip-adress').val(),
        });
        
        // on connection start listening
        ros.on('connection', function () {
            console.log('Connected and publishing');

            pub.publish(mess);

            var listener = new ROSLIB.Topic({
                ros: ros,
                name: '/rosout_agg',
                messageType: 'rosgraph_msgs/Log',
            });
            
            // on connection subscribe messages from ROS core
            listener.subscribe(function (message) {
                console.log(message.msg);
                // if Failed to connect robot -> reload page
                if(message.msg.includes('Failed to connect to robot')){   
                    alert('Check IP adress or type of robot! Reload page after 5 seconds!', 'danger')
                    window.setTimeout(function () {
                        window.location.href = '/';
                    }, 5000); //5seonds
                    
                }
                
                // If is connected -> wait on hit button Play in UR Polyscope
                else if(message.msg.includes('Robot connected to reverse interface.')){
                    alert('Robot is ready! Running Moveit! Redirect page after 5 seconds!', 'success')
            
                    var mess2 = new ROSLIB.Message({
                        data: "moveit_run"
                    });
                    // publish to run moveit
                    pub.publish(mess2);

                    window.setTimeout(function () {
                        window.location.href = '/robot_control'
                    }, 5000); //5seonds
                }
            });
        });
    });
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