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
public class GpsMeasurement extends BaseMessage implements Message {
	public static final String version_id = "gpsV0.1";

	public Date timestamp;

	public double latitude;
	public boolean north;
	public double longitude;
	public boolean west;

	public GpsMeasurement(double latitude, double longitude) {
		this.timestamp = new Date();
		this.latitude = latitude;
		this.longitude = longitude;
	}

	@Override
	public String toLogString() {
		String s = super.formatter.format(timestamp);
		
		s += ',' + Double.toString(latitude);
		if (north) s += ",N";
		else s += ",S";
		
		s += ',' + Double.toString(longitude);
		if (west) s += ",W";
		else s += ",E";
		
		return s;
	}

	@Override
	public Message fromLogString(String str) {
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

		latitude = Double.parseDouble(ar[1]);
		north = ar[2].equalsIgnoreCase("N");
		longitude = Double.parseDouble(ar[3]);
		west = ar[3].equalsIgnoreCase("W");
		return new GpsMeasurement(latitude, longitude);
	}
}