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
public class EncoderMeasurement extends BaseMessage implements Message {

	public static final String version_id = "encoderV0.0";

	public Date timestamp;
	public double distance;
	public double velocity;
	public double dataWord;
	public double accel;

	// Makes an encoder measurement with the time of Now.
	public EncoderMeasurement(double distance, double velocity) {
		this.distance = distance;
		this.velocity = velocity;
		this.timestamp = new Date();
		this.dataWord = 0;
		this.accel = 0;
	}
	
	public EncoderMeasurement(Date timestamp, double distance, double velocity) {
		this.timestamp = timestamp;
		this.distance = distance;
		this.velocity = velocity;
		this.dataWord = 0;
		this.accel = 0;
	}
	
	//logs the number of ticks received since last time as well
	public EncoderMeasurement(Date timestamp, double dataWord, double distance, double velocity, double accel) {
		this.timestamp = timestamp;
		this.dataWord = dataWord;
		this.distance = distance;
		this.velocity = velocity;
		this.accel = accel;		
	}

	@Override
	public String toLogString() {
		return super.formatter.format(timestamp) + ','
				+ Double.toString(dataWord) + ','
				+ Double.toString(distance) + ',' 
				+ Double.toString(velocity) + ','
				+ Double.toString(accel);
	}

	@Override
	public Message fromLogString(String str) {
		String[] ar = str.split(",");
		Date d = try_to_parse_date(ar[0]);
		distance = Double.parseDouble(ar[1]);
		velocity = Double.parseDouble(ar[2]);
		dataWord = Double.parseDouble(ar[3]);
		//TODO calculate acceleration
		return new EncoderMeasurement(timestamp, distance, velocity, dataWord, accel);
	}
}