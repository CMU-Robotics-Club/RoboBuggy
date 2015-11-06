package com.roboclub.robobuggy.messages;

import java.text.ParseException;
import java.util.Date;

import com.roboclub.robobuggy.main.config;
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
public class ImuMeasurement extends BaseMessage implements Message {

	public static final String version_id = "imuV0.0";

	public Date timestamp;

	public double yaw;
	public double pitch;
	public double roll;

	public ImuMeasurement(double y, double p, double r) {
		this.timestamp = new Date();
		this.yaw = y;
		this.pitch = p;
		this.roll = r;
	}

	public String toLogString() {
		String s = super.formatter.format(timestamp);
		return s + "," + Double.toString(yaw) + "," 
				+ Double.toString(pitch) + "," 
				+ Double.toString(roll);
	}

	public Message fromLogString(String str) {
		String delims = ",";
		String[] ar = str.split(delims);

		// Creating SimpleDateFormat with yyyyMMdd format e.g."20110914"
		String yyyyMMdd = ar[0];
		try {
			timestamp = super.formatter.parse(yyyyMMdd);
		} catch (ParseException e) {
			e.printStackTrace();
		}

		Double y = Double.parseDouble(ar[1]);
		Double p = Double.parseDouble(ar[2]);
		Double r = Double.parseDouble(ar[3]);
		return new ImuMeasurement(y, p, r);
	}

	@Override
	public String getCorrespondingSensor() {
		// TODO Auto-generated method stub
		return config.SENSOR_NAME_IMU;
	}
}
