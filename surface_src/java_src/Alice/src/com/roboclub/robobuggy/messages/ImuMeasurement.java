package com.roboclub.robobuggy.messages;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * @author ?
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

// Represents raw measurement from the IMU
public class ImuMeasurement extends BaseMessage implements Message {

	public static final String version_id = "imuV0.0";

	public Date timestamp;

	public float aX;
	public float aY;
	public float aZ;
	public float rX;
	public float rY;
	public float rZ;
	public float mX;
	public float mY;
	public float mZ;

	public ImuMeasurement(float aX, float aY, float aZ, float rX, float rY,
			float rZ, float mX, float mY, float mZ) {
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
		return s + ',' + Float.toString(aX) + ',' + Float.toString(aY) + ','
				+ Float.toString(aZ) + ',' + Float.toString(rX) + ','
				+ Float.toString(rY) + ',' + Float.toString(rZ) + ','
				+ Float.toString(mX) + ',' + Float.toString(mY) + ','
				+ Float.toString(mZ);
	}

	@Override
	public Message fromLogString(String str) {
		String delims = ",";
		String[] ar = str.split(delims);

		String yyyyMMdd = ar[0];
		formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		try {
			timestamp = formatter.parse(yyyyMMdd);
		} catch (ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		Float aX = Float.parseFloat(ar[1]);
		Float aY = Float.parseFloat(ar[2]);
		Float aZ = Float.parseFloat(ar[3]);
		Float rX = Float.parseFloat(ar[4]);
		Float rY = Float.parseFloat(ar[5]);
		Float rZ = Float.parseFloat(ar[6]);
		Float mX = Float.parseFloat(ar[7]);
		Float mY = Float.parseFloat(ar[8]);
		Float mZ = Float.parseFloat(ar[9]);
		return new ImuMeasurement(aX, aY, aZ, rX, rY, rZ, mX, mY, mZ);
	}
}
