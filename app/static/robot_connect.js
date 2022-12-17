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

$(".ur").click(function(event){
    $('#modal-title').text(event.target.id)
    console.log(event.target.id);
});

$('#ip-adress, #ros-ip-adress').inputmask({
    alias: "ip",
    greedy: false
});

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

$('#connect').click(function () {
    var ros = new ROSLIB.Ros();

    // connect to ROS
    ros.connect('ws://' + $('#ros-ip-adress').val() + ':9090');

    ros.on('connection', function () {
        alert('Succesfull! Connection to Server! Connecting to robot!', 'success')
        Cookies.set('ros_ip_adress', $('#ros-ip-adress').val())
    });
    ros.on('close', function () {
        alert('Connection to Server closed!', 'warning')
    });
    ros.on('error', function () {
        alert('Connection to Server failed! Reload page after 5 seconds!', 'danger')
        window.setTimeout(function () {
            window.location.href = '/';
        }, 5000); //5seonds
    });

    var pub = new ROSLIB.Topic({
        ros: ros,
        name: '/switch',
        messageType: 'std_msgs/String',
    });

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
        
        listener.subscribe(function (message) {
            console.log(message.msg);
            if(message.msg.includes('Failed to connect to robot')){   
                alert('Check IP adress or type of robot! Reload page after 5 seconds!', 'danger')
                window.setTimeout(function () {
                    window.location.href = '/';
                }, 5000); //5seonds
                
            }
            
            else if(message.msg.includes('Robot connected to reverse interface.')){
                alert('Robot is ready! Running Moveit! Redirect page after 5 seconds!', 'success')
        
                var mess2 = new ROSLIB.Message({
                    data: "moveit_run"
                });
                pub.publish(mess2);

                window.setTimeout(function () {
                    window.location.href = '/robot_control'
                }, 5000); //5seonds
            }
        });
    });
});