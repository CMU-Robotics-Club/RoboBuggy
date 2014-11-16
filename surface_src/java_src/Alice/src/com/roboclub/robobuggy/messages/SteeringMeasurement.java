package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

public class SteeringMeasurement implements Message {
	private int angle;
	private Date timestamp;
	
	public SteeringMeasurement(int angle) {
		this.timestamp = new Date();
		this.angle = angle;
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
