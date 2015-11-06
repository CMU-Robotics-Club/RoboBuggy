package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * @author ?
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

// Represents raw measurement from the IMU
public class WheelAngleCommand implements Message {
	// V0.0 had int
	// V0.1 has float. Note that round-off is hundreths of a degree.
	public static final String version_id = "autonomous_angleV0.1";

	public Date timestamp;
	public float angle;

	// Makes an encoder measurement with the time of Now.
	public WheelAngleCommand(float angle) {
		this.angle = angle;
		this.timestamp = new Date();
	}

	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Message fromLogString(String str) {
		// TODO Auto-generated method stub
		return null;

	}

	@Override
	public String getCorrespondingSensor() {
		// TODO Auto-generated method stub
		return null;
	}

}