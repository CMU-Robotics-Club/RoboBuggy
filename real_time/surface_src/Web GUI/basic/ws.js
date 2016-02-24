window.onload = read();

function read() {
    var socket = new WebSocket("ws://127.0.0.1:8080");
    socket.onmessage = onMessage;
    socket.onopen = function() {
        socket.send("Please add me to the data group!");    
    }
}

function updateIMU(msg) {
    console.log(msg);
    document.getElementById("roll").innerHTML = msg.roll.toString();
    document.getElementById("pitch").innerHTML = msg.pitch.toString();
    document.getElementById("yaw").innerHTML = msg.yaw.toString();
}

function updateEncoder(msg) {
    console.log(msg);
    document.getElementById("dist").innerHTML = msg.distance.toString();
    document.getElementById("vel").innerHTML = msg.velocity.toString();
    document.getElementById("acc").innerHTML = msg.accel.toString();
}

function updateSteering(msg) {
    console.log(msg);
    document.getElementById("steering").innerHTML = msg.angle.toString();
}

function onMessage(event) {

    // if this is an image
    if (event.data.indexOf("data:image/png;base64,") > -1) {
        document.getElementById('img').setAttribute('src', event.data);
        console.log("updating image");
        return;
    }

    console.log(event.data);
    // otherwise
    var msg = JSON.parse(event.data);
    var name = msg.VERSION_ID.toLowerCase();

    if (name.indexOf("imu") > -1) {
        updateIMU(msg);
    } else if (name.indexOf("steering") > -1) {
        updateSteering(msg);
    } else if (name.indexOf("encoder") > -1) {
        updateEncoder(msg);
    } else {
        return;
    }
}
