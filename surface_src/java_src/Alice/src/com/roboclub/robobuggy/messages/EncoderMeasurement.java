package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

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
		return String.format("%s,%s\n", format_the_date(timestamp),
				String.valueOf(distance), String.valueOf(velocity));
	}

	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = try_to_parse_date(spl[0]);
		double distance = Double.parseDouble(spl[1]);
		double velocity = Double.parseDouble(spl[2]);
		return new EncoderMeasurement(d, distance, velocity);
	}

}