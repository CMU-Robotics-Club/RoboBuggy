window.onload = read();

function read() {
    var socket = new WebSocket("ws://127.0.0.1:8080");
    socket.onmessage = onMessage;
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
    console.log(event.data);
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
