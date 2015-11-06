package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.ros.Message;

public class SteeringMeasurement extends BaseMessage implements Message {
	public int angle;
	private Date timestamp;
	
	public SteeringMeasurement(int angle) {
		this.timestamp = new Date();
		this.angle = angle;
	}
	
	@Override
	public String toLogString() {
		String s = super.formatter.format(timestamp);
		return s + ',' + Double.toString(angle);
	}

	@Override
	public Message fromLogString(String str) {
		String delims = ",";
		String[] ar = str.split(delims);
		timestamp = try_to_parse_date(ar[0]);
		angle = Integer.parseInt(ar[1]);
		return new SteeringMeasurement(angle);
	}

	@Override
	public String getCorrespondingSensor() {
		// TODO Auto-generated method stub
		return config.SENSOR_NAME_STEERING;
	}
}
