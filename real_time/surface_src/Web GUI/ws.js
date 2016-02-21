window.onload = read();

function read() {
    var socket = new WebSocket("ws://127.0.0.1:8080");
    socket.onmessage = onMessage;
}

function updateIMU(msg) {
    console.log(msg);
    document.getElementById("roll").innerHTML = msg.params.roll.toString();
    document.getElementById("pitch").innerHTML = msg.params.pitch.toString();
    document.getElementById("yaw").innerHTML = msg.params.yaw.toString();
}

function updateEncoder(msg) {
    console.log(msg);
}

function onMessage(event) {
    var msg = JSON.parse(event.data);
    switch (msg.name.toLowerCase()) {
        case "imu":
            updateIMU(msg);
            break;
        case "encoder":
            updateEncoder(msg);
            break;
        default:
            break;
    }
}
