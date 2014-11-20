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
public class ImuMeasurement implements Message {

	public Date timestamp;

	public double aX;
	public double aY;
	public double aZ;
	public double rX;
	public double rY;
	public double rZ;
	public double mX;
	public double mY;
	public double mZ;

	public ImuMeasurement(double aX, double aY, double aZ, double rX, double rY,
			double rZ, double mX, double mY, double mZ) {
		timestamp = new Date();
		
		this.aX = aX;
		this.aY = aY;
		this.aZ = aZ;
		this.rX = rX;
		this.rY = rY;
		this.rZ = rZ;
		this.mX = mX;
		this.mY = mY;
		this.mZ = mZ;
	}

	@Override
	public String toLogString() {
		String s = formatter.format(timestamp);
		return s + ',' + Double.toString(aX) + ',' + Double.toString(aY) + ','
				+ Double.toString(aZ) + ',' + Double.toString(rX) + ','
				+ Double.toString(rY) + ',' + Double.toString(rZ) + ','
				+ Double.toString(mX) + ',' + Double.toString(mY) + ','
				+ Double.toString(mZ);
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

		aX = Double.parseDouble(ar[1]);
		aY = Double.parseDouble(ar[2]);
		aZ = Double.parseDouble(ar[3]);
		rX = Double.parseDouble(ar[4]);
		rY = Double.parseDouble(ar[5]);
		rZ = Double.parseDouble(ar[6]);
		mX = Double.parseDouble(ar[7]);
		mX = Double.parseDouble(ar[8]);
		mX = Double.parseDouble(ar[9]);

	}
}
