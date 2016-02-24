/* @file index.js
 * @brief Draws some nifty stuff.
 */
'use strict';

/* @brief Implements a viewer to render viewing roll/pitch/yaw.  */
class AngleViewer {
  /* @brief Makes a new AngleViewer, given the id of the canvas to render in. */
  constructor(id, c_x, c_y, c_z, title) {
    this.div = document.getElementById(id);
    this.canvas = document.createElement('canvas');
    this.canvas.height = this.div.clientHeight;
    this.canvas.width = this.div.clientWidth;
    this.div.appendChild(this.canvas);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(100, this.canvas.width / this.canvas.height, 0.1, 1000);

    var zoom = 2;
    this.camera.position.x = zoom * c_x;
    this.camera.position.y = zoom * c_y;
    this.camera.position.z = zoom * c_z;

    this.camera.lookAt(new THREE.Vector3(0.0, 0.0, 0.0));

    this.renderer = new THREE.WebGLRenderer({
      'canvas': this.canvas,
    });
    this.renderer.setSize(this.canvas.height, this.canvas.width);

    this.robot = this.createRobot(3, 1, 1);
    this.scene.add(this.robot);

    this.createAxes();
    this.createTextOverlay(title);

    this.setAngles(0, 0, 0);
    this.timer = requestAnimationFrame(this.rerender.bind(this));
  }

  createRobot(robot_x, robot_y, robot_z) {
    // Chassis.
    var chassis_geometry = new THREE.BoxGeometry(robot_x, robot_y, robot_z);
    var colors = [ 0xff0000, 0x0000ff, 0x00ff00];
    for (var i = 0; i < chassis_geometry.faces.length; i++) {
      var color = colors[Math.floor(i / 4)];
      chassis_geometry.faces[i].color.setHex(color);
    }

    var chassis_material = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      vertexColors: THREE.FaceColors
    });

    var chassis_mesh = new THREE.Mesh(chassis_geometry, chassis_material);

    var robot = new THREE.Object3D();
    robot.add(chassis_mesh);

    // Axes.
    var axis_length = 0.5;
    var axis_color = 0xffffff;
    var axis_arrow_length = 0.10;
    var axis_arrow_width = 0.10;
    var origin = new THREE.Vector3(0, 0, 0);

    var new_arrow = function (dir, pos) {
      var arrow = new THREE.ArrowHelper(
        dir, pos,
        axis_length, axis_color,
        axis_arrow_length, axis_arrow_width);
        robot.add(arrow);
    }.bind(this);

    var x_axis_dir = new THREE.Vector3(1, 0, 0);
    var x_axis_pos = new THREE.Vector3(robot_x / 2, 0, 0);
    new_arrow(x_axis_dir, x_axis_pos);

    var y_axis_dir = new THREE.Vector3(0, 1, 0);
    var y_axis_pos = new THREE.Vector3(0, robot_y / 2, 0);
    new_arrow(y_axis_dir, y_axis_pos);

    var z_axis_dir = new THREE.Vector3(0, 0, 1);
    var z_axis_pos = new THREE.Vector3(0, 0, robot_z / 2);
    new_arrow(z_axis_dir, z_axis_pos);

    return robot;
  }

  createAxes() {
    var axis_length = 2.5;
    var axis_color = 0xffffff;
    var axis_arrow_length = 0.10;
    var axis_arrow_width = 0.10;
    var origin = new THREE.Vector3(0, 0, 0);

    var new_arrow = function (dir) {
      var arrow = new THREE.ArrowHelper(
        dir, origin,
        axis_length, axis_color,
        axis_arrow_length, axis_arrow_width);
        this.scene.add(arrow);
    }.bind(this);

    var x_axis_dir = new THREE.Vector3(1, 0, 0);
    new_arrow(x_axis_dir);
    var y_axis_dir = new THREE.Vector3(0, 1, 0);
    new_arrow(y_axis_dir);
    var z_axis_dir = new THREE.Vector3(0, 0, 1);
    new_arrow(z_axis_dir);
  }

  createTextOverlay(title) {
    var title_div = document.createElement('div');
    title_div.innerHTML = title;
    title_div.className = 'title';
    this.div.appendChild(title_div);

    // Display Roll/Pitch/Yaw in degrees in the bottom of the div.
    var info_div = document.createElement('div');
    info_div.className = 'info';

    var roll_div = document.createElement('div');
    roll_div.innerHTML = 'Roll: ';
    info_div.appendChild(roll_div);

    var pitch_div = document.createElement('div');
    pitch_div.innerHTML = 'Pitch: ';
    info_div.appendChild(pitch_div);

    var yaw_div = document.createElement('div');
    yaw_div.innerHTML = 'Yaw: ';
    info_div.appendChild(yaw_div);

    this.div.appendChild(info_div);

    // Update the text.
    var toDeg = function (rad) { return rad * 180 / Math.PI; }

    this.updateText = function (roll, pitch, yaw) {
      roll_div.innerHTML = `Roll: ${toDeg(roll).toFixed(2)}&deg;`;
      pitch_div.innerHTML = `Pitch: ${toDeg(pitch).toFixed(2)}&deg;`;
      yaw_div.innerHTML = `Yaw: ${toDeg(yaw).toFixed(2)}&deg;`;
    }
  }

  /* @brief Given 3 angles in radians, sets them to be redrawn in the next
   * rerender.
   */
  setAngles(roll, pitch, yaw) {
    this.robot.rotation.x = roll;
    this.robot.rotation.y = pitch;
    this.robot.rotation.z = yaw;

    this.updateText(roll, pitch, yaw);

    this.hasChanged = true;
  }

  /* @brief Clears the render canvas. */
  clear() {
    this.context.clearRect(
      -1 * this.canvas.width / 2,
      -1 * this.canvas.height / 2,
      this.canvas.width, this.canvas.height);
  }

  rerender() {
    if (!this.hasChanged)  {
      this.timer = requestAnimationFrame(this.rerender.bind(this));
      return;
    }

    this.renderer.render(this.scene, this.camera);

    this.hasChanged = false;
    this.timer = requestAnimationFrame(this.rerender.bind(this));
  }

  destroy() {
    cancelAnimationFrame(this.timer);
  }
}

var imu1 = new AngleViewer('imu-canvas1', 1, 1, 1, 'Top Right View');
var imu2 = new AngleViewer('imu-canvas2', 1, 1, 0, 'Top Front View');
var imu3 = new AngleViewer('imu-canvas3', 0, 0, 1, 'Side View');
var imu4 = new AngleViewer('imu-canvas4', 1, -1, 1, 'Bottom Front View');

class GPSViewer {
  constructor(id) {
    this.div = document.getElementById(id);

    this.map = new google.maps.Map(this.div, {
      'zoom': 18,
      'center': { lat: 40.442850666666665, lng: -79.94273116666666},
      // 'mapTypeId': google.maps.MapTypeId.TERRAIN,
    });

    var overlay = new google.maps.OverlayView();

    // Add the container when the overlay is added to the map.
    overlay.onAdd = function() {
      console.log('ay');
      var layer = d3.select(this.getPanes().overlayLayer).append("div")
      .attr("class", "stations");

      overlay.draw = function() {
        var projection = this.getProjection();
        var padding = 10;

        var marker = layer.selectAll("svg")
          .data(d3.entries(window.gpsdata))
          .each(transform) // update existing markers
          .enter().append("svg")
          .each(transform)
          .attr("class", "marker");

        // Add a circle.
        marker.append("circle")
        .attr("r", 4.5)
        .attr("cx", padding)
        .attr("cy", padding);

        function transform(d) {
          // gotta convert to West
          d = new google.maps.LatLng(d.value.latitude, 360 - d.value.longitude);
          d = projection.fromLatLngToDivPixel(d);
          return d3.select(this)
          .style("left", (d.x - padding) + "px")
          .style("top", (d.y - padding) + "px");
        }
      };

    }

    overlay.setMap(this.map);

    this.hasChanged = false;
    // this.timer = requestAnimationFrame(this.rerender.bind(this));
  }

  rerender() {
    if (!this.hasChanged)  {
      this.timer = requestAnimationFrame(this.rerender.bind(this));
      return;
    }

    this.hasChanged = false;
    this.timer = requestAnimationFrame(this.rerender.bind(this));
  }

  destroy() {
    cancelAnimationFrame(this.timer);
  }
}

var gps = new GPSViewer('gps-canvas');

var idx = 0;
var deg2rad = function (d) { return d * Math.PI / 180; }

// setInterval(function () {
//   var r = deg2rad(imudata[idx].roll);
//   var p = deg2rad(imudata[idx].pitch);
//   var y = deg2rad(imudata[idx].yaw);
//   idx = (idx + 1) % imudata.length;

//   imu1.setAngles(r, p, y);
//   imu2.setAngles(r, p, y);
//   imu3.setAngles(r, p, y);
//   imu4.setAngles(r, p, y);
// }, 10);
