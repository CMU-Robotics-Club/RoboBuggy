package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

// Represents raw measurement from the IMU
public class EncoderMeasurement implements Message {

	public Date timestamp;
	public double distance;
	public double velocity;

	// Makes an encoder measurement with the time of Now.
	public EncoderMeasurement(double distance, double velocity) {
		this.distance = distance;
		this.velocity = velocity;
		this.timestamp = new Date();
	}
	

}