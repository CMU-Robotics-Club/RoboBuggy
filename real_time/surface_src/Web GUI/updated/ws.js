window.onload = read();

function read() {
    imu1 = new AngleViewer('imu-canvas1', 1, 1, 1, 'Top Right View');
    imu2 = new AngleViewer('imu-canvas2', 1, 1, 0, 'Top Front View');
    imu3 = new AngleViewer('imu-canvas3', 0, 0, 1, 'Side View');
    imu4 = new AngleViewer('imu-canvas4', 1, -1, 1, 'Bottom Front View');

    var dataSocket = new ReconnectingWebSocket("ws://buggy:8080");
    dataSocket.onmessage = onDataMessage;
    dataSocket.onopen = function() {
        dataSocket.send("Group: data");
    }

    document.getElementById("start-button").onclick = function() {
        localDataSocket.send("Command: herp");
    }

    var localDataSocket = new ReconnectingWebSocket("ws://127.0.0.1:8080");
    localDataSocket.onmessage = onDataMessage;
    localDataSocket.onopen = function() {
        localDataSocket.send("Group: data");
    }

    cs = new CameraStream("camera");
    var cameraSocket = new ReconnectingWebSocket("ws://buggy:8080", null,
        {binaryType: 'arraybuffer'});
    cameraSocket.onmessage = function(event) { cs.handlePacket(event); };
    cameraSocket.onopen = function() { cameraSocket.send("Group: camera"); };

    var localCameraSocket = new ReconnectingWebSocket("ws://127.0.0.1:8080", null,
        {binaryType: 'arraybuffer'});
    localCameraSocket.onmessage = function(event) { cs.handlePacket(event); };
    localCameraSocket.onopen = function() { localCameraSocket.send("Group: camera"); };

    distanceGauge = new JustGage({
        id: "gauge-distance",
        value: 0,
        min: 0,
        max: 1500,
        title: "Distance"
    });

    velocityGauge = new JustGage({
        id: "gauge-velocity",
        value: 69,
        min: 0,
        max: 100,
        title: "Velocity"
    });

    accelerationGauge = new JustGage({
        id: "gauge-acceleration",
        value: 77,
        min: 0,
        max: 100,
        title: "Acceleration"
    });
}

function updateIMU(msg) {
    var r = deg2rad(msg.roll);
    var p = deg2rad(msg.pitch);
    var y = deg2rad(msg.yaw);

    imu1.setAngles(r, p, y);
    imu2.setAngles(r, p, y);
    imu3.setAngles(r, p, y);
    imu4.setAngles(r, p, y);
}

function updateEncoder(msg) {
    console.log(msg);
    distanceGauge.refresh(msg.distance, null);
    velocityGauge.refresh(msg.velocity, null);
    accelerationGauge.refresh(msg.accel, null);
    // document.getElementById("dist").innerHTML = msg.distance.toString();
    // document.getElementById("vel").innerHTML = msg.velocity.toString();
    // document.getElementById("acc").innerHTML = msg.accel.toString();
}

function updateSteering(msg) {
    // console.log(msg);
    // document.getElementById("steering").innerHTML = msg.angle.toString();
}

function updateLogging(msg) {
    var ta = document.getElementById("output");
    ta.value += msg.message + "\n";
    ta.scrollTop = ta.scrollHeight;
}

function onDataMessage(event) {

    var msg;
    // We might sometimes get invalid json parses since there are NaN and Inf
    try {
        msg = JSON.parse(event.data);
    } catch (err) {
        return;
    }

    var name = msg.VERSION_ID.toLowerCase();

    if (name.indexOf("imu") > -1) {
        updateIMU(msg);
    //} else if (name.indexOf("steering") > -1) {
        //updateSteering(msg);
    } else if (name.indexOf("encoder") > -1) {
        updateEncoder(msg);
    } else if (name.indexOf("camera") > -1) {
        return;
    } else if (name.indexOf("logic_notification") > -1) {
        updateLogging(msg);
    } else {
        console.log(msg);
    }
}
