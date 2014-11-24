package com.roboclub.robobuggy.messages;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
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

	// Makes an encoder measurement with the time of Now.
	public EncoderMeasurement(double distance, double velocity) {
		this.distance = distance;
		this.velocity = velocity;
		this.timestamp = new Date();
	}

	public EncoderMeasurement(Date timestamp, double distance, double velocity) {
		this.timestamp = timestamp;
		this.distance = distance;
		this.velocity = velocity;
	}

	@Override
	public String toLogString() {
		String s = super.formatter.format(timestamp);
		return s + ',' + Double.toString(velocity) + ',' 
				+ Double.toString(distance);
	}

	@Override
	public Message fromLogString(String str) {
		String[] ar = str.split(",");
		Date d = try_to_parse_date(ar[0]);
		distance = Double.parseDouble(ar[1]);
		velocity = Double.parseDouble(ar[2]);
		return new EncoderMeasurement(timestamp, distance, velocity);
	}
}