window.onload = read();

var width=1280;
var height=720;

var c=document.getElementById("camera-pushbar-canvas");
var ctx=c.getContext("2d");
var imgData=ctx.createImageData(1440,900);

var blob;
var readSize=0;
var started=false;
var imageSize=0;
var image64="";

function read() {
    var dataSocket = new WebSocket("ws://127.0.0.1:8080");
    dataSocket.onmessage = onDataMessage;
    dataSocket.onopen = function() {
        dataSocket.send("Please add me to the data group!");    
    }

    var cameraSocket = new WebSocket("ws://127.0.0.1:8080");
    cameraSocket.binaryType = "arraybuffer";
    cameraSocket.onmessage = onCameraMessage;
    cameraSocket.onopen = function() {
        cameraSocket.send("I wanna see the freaking camera plz");
    }
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
    // console.log(msg);
    // document.getElementById("dist").innerHTML = msg.distance.toString();
    // document.getElementById("vel").innerHTML = msg.velocity.toString();
    // document.getElementById("acc").innerHTML = msg.accel.toString();
}

function updateSteering(msg) {
    // console.log(msg);
    // document.getElementById("steering").innerHTML = msg.angle.toString();
}

function onDataMessage(event) {

    // We might sometimes get invalid json parses since there are NaN and Inf
    try {
        var msg = JSON.parse(event.data);    
    } catch (err) {
        return;
    }
    
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

function onCameraMessage(event) {

    blob=event;
    var r=String.fromCharCode.apply(null, new Uint8Array(blob.data));
     

    if (started==false){
            if (r==9){
                    imageSize=parseInt(r);
            }else{
                    imageSize=parseInt(r.substring(0,9));
                    image64+=r.substring(9);
                    readSize+=parseInt(r.length-9);
            }
            started=true;
    }else{
            image64+=r;
            readSize+=parseInt(r.length);
    }
    if (readSize==imageSize){
            img=new Image();
        img.onload=function(){
            ctx.drawImage(img,0,0,img.width,img.height,0,0,width,height);
        }
            img.src='data:image/jpg;base64,'+image64;

            readSize=0;
            started=false;
            image64="";
    }
}


























