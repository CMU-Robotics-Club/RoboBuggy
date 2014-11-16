package com.roboclub.robobuggy.messages;

import java.text.DateFormat;
import java.text.Format;
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

	Format formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");

	public ImuMeasurement(double aX, double aY, double aZ, double rX, double rY,
			double rZ, double mX, double mY, double mZ) {
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

		aX = Float.parseFloat(ar[1]);
		aY = Float.parseFloat(ar[2]);
		aZ = Float.parseFloat(ar[3]);
		rX = Float.parseFloat(ar[4]);
		rY = Float.parseFloat(ar[5]);
		rZ = Float.parseFloat(ar[6]);
		mX = Float.parseFloat(ar[7]);
		mX = Float.parseFloat(ar[8]);
		mX = Float.parseFloat(ar[9]);

	}
}
