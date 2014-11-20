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

	@Override
	public String toLogString() {
		String s = formatter.format(timestamp);
		return s + ',' + Double.toString(velocity) + ',' 
				+ Double.toString(distance);
	}

	@Override
	public void fromLogString(String str) {
		String delims = ",";
		String[] ar = str.split(delims);

		DateFormat formatter = null;
		// Creating SimpleDateFormat with yyyyMMdd format e.g."20110914"
		String yyyyMMdd = ar[0];
		formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		try {
			timestamp = (Date) formatter.parse(yyyyMMdd);
		} catch (ParseException e) {
			e.printStackTrace();
		}

		distance = Double.parseDouble(ar[1]);
		velocity = Double.parseDouble(ar[2]);
	}

}