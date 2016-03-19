'use strict';

class CameraStream {
    constructor(id) {
        //this.div = document.getElementById(id);
        //console.log(this.div.height);
        //console.log(this.div.width);

        //this.canvas = document.createElement('canvas');
        //this.canvas.height = this.div.getAttribute("height");
        //this.canvas.width = this.div.getAttribute("width");
        //this.div.appendChild(this.canvas);

        this.canvas = document.getElementById(id);

        this.currentString = "";
        var ctx = this.canvas.getContext("2d");
        ctx.fillStyle = "white";
        ctx.font = "30px Arial";
        ctx.textAlign = "left";
        this.cameraImage = new Image();
        this.frameNumber = -1;

        // scoping issues with 'this', lambda function temporary fix.
        this.cameraImage.onload = () => {
            ctx.drawImage(this.cameraImage, 0, 0, this.canvas.width,
                          this.canvas.height);
            ctx.fillText(this.frameNumber.toString(), 7, 30);
        };
    }

    bytesToBase64(arr) {
        var col = [];
        const step = 0x8000;
        for (var sliceStart = 0; sliceStart < arr.length; sliceStart += step) {
            col.push(String.fromCharCode.apply(null, arr.subarray(sliceStart, sliceStart + step)));
        }
        return col.join("");
    }

    handlePacket(packet) {
        /* Header Format
         * [10 chars] Frame Number
         * [10 chars] Image Base 64 String Size in bytes
         */
        const headerSize = 20; // Bytes

        // var packetString = StringView.bytesToBase64(new Uint8Array(packet.data));
        //var packetString = String.fromCharCode.apply(null, new Uint8Array(packet.data));
        var packetString = this.bytesToBase64(new Uint8Array(packet.data));
        this.currentString += packetString;

        // the first [headersize] bytes of the current string will always
        // be a valid header, if they exist.
        // should probably put a try/except here incase we get an invalid header
        if (this.currentString.length >= headerSize) {
            this.frameNumber = parseInt(this.currentString.substring(0,10));
            var dataSize = parseInt(this.currentString.substring(10, 20));

            if ((this.currentString.length - headerSize) >= dataSize) {
                this.cameraImage.src = ("data:image/png;base64," +
                        this.currentString.substring(headerSize, headerSize + dataSize));
                this.currentString = this.currentString.substring(headerSize + dataSize);
            }
        }
    }
}

window.onload = function() {
    var cs = new CameraStream("camera");
    var cs1 = new CameraStream("camera1");
    var cs2 = new CameraStream("camera2");
    var cameraSocket = new ReconnectingWebSocket("ws://127.0.0.1:8080", null,
        {binaryType: 'arraybuffer'});
    // var cameraSocket = new ReconnectingWebSocket("ws://127.0.0.1:8080");
    // cameraSocket.debug = false;
    // cameraSocket.reconnectInterval = 1000;
    // cameraSocket.binaryType = "arraybuffer";
    cameraSocket.onmessage = function(event) {
        cs.handlePacket(event);
        cs1.handlePacket(event);
        cs2.handlePacket(event);
    };
    cameraSocket.onopen = function() {
        cameraSocket.send("I wanna see camera pls");
    };
};
