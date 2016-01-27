package com.roboclub.robobuggy.localization;

/**
 * Class used to represent a coordinate frame
 */
public class CoordinateFrame {
	// TODO
}

/*
 * location of frames on the buggy Buggy frame = front wheel and servo
 * intersection gps => x:-8in y:0 z: 4in q0:1 q1:0 q2: 0 q3: 0 cam1 => x: 3in
 * y:0 z:7.5in q0:1 q1:0 q2: 0 q3: 0 cam2 => ? imu => ? odom => x: 0in y:0in z:
 * 0in q0: 1 q1:0 q2: 0 q3: 0
 * 
 * H_gps_to_wheel = [cos(th) -sin(th) dx; sin(th) cos(th) dy; 0 0 1]
 */
