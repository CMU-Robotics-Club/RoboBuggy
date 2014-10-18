package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * @author ?
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

// Represents raw measurement from the IMU
public class WheelAngleCommand implements Message {

	public Date timestamp;
	public double angle;

	// Makes an encoder measurement with the time of Now.
	public WheelAngleCommand(double angle) {
		this.angle = angle;
		this.timestamp = new Date();
	}

	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void fromLogString(String str) {
		// TODO Auto-generated method stub

	}

}