package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

// Represents raw measurement from the IMU
public class BrakeCommand implements Message {

	public Date timestamp;
	public boolean down;

	// Makes an encoder measurement with the time of Now.
	public BrakeCommand(boolean brake_is_down) {
		this.down = brake_is_down;
		this.timestamp = new Date();
	}

	@Override
	public String toLogString() {
		// TODO Auto-generated method stub

	}

	@Override
	public void fromLogString() {
		// TODO Auto-generated method stub
		return null;
	}
}