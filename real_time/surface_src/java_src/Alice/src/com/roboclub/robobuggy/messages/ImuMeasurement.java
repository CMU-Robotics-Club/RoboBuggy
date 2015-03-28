package com.roboclub.robobuggy.messages;

import java.text.ParseException;
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
public class ImuMeasurement extends BaseMessage implements Message {

	public static final String version_id = "imuV0.0";

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

	public ImuMeasurement(double aX2, double aY2, double aZ2, double rX2,
			double rY2, double rZ2, double mX2, double mY2, double mZ2) {
		this.timestamp = new Date();
		this.aX = aX2;
		this.aY = aY2;
		this.aZ = aZ2;
		this.rX = rX2;
		this.rY = rY2;
		this.rZ = rZ2;
		this.mX = mX2;
		this.mY = mY2;
		this.mZ = mZ2;
	}

	@Override
	public String toLogString() {
		String s = super.formatter.format(timestamp);
		return s + ',' + Double.toString(aX) + ',' + Double.toString(aY) + ','
				+ Double.toString(aZ) + ',' + Double.toString(rX) + ','
				+ Double.toString(rY) + ',' + Double.toString(rZ) + ','
				+ Double.toString(mX) + ',' + Double.toString(mY) + ','
				+ Double.toString(mZ);
	}

	@Override
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

		Double aX = Double.parseDouble(ar[1]);
		Double aY = Double.parseDouble(ar[2]);
		Double aZ = Double.parseDouble(ar[3]);
		Double rX = Double.parseDouble(ar[4]);
		Double rY = Double.parseDouble(ar[5]);
		Double rZ = Double.parseDouble(ar[6]);
		Double mX = Double.parseDouble(ar[7]);
		Double mY = Double.parseDouble(ar[8]);
		Double mZ = Double.parseDouble(ar[9]);
		return new ImuMeasurement(aX, aY, aZ, rX, rY, rZ, mX, mY, mZ);
	}
}
