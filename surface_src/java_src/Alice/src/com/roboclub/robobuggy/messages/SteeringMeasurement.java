package com.roboclub.robobuggy.messages;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

public class SteeringMeasurement implements Message {
	public int angle;
	private Date timestamp;
	
	public SteeringMeasurement(int angle) {
		this.timestamp = new Date();
		this.angle = angle;
	}
	
	@Override
	public String toLogString() {
		String s = formatter.format(timestamp);
		return s + ',' + Double.toString(angle);
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

		angle = Integer.parseInt(ar[1]);
	}
}
