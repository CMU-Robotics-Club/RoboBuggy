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
	public static final String version_id = "autonomous_angleV0.0";

	public Date timestamp;
	public int angle;

	// Makes an encoder measurement with the time of Now.
	public WheelAngleCommand(int angle) {
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

}