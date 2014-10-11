package com.roboclub.robobuggy.messages;

import java.text.Format;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

// Represents raw measurement from the IMU
public class ImuMeasurement implements Message {
	
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
	

	Format formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");

	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		
		String s = formatter.format(timestamp);
		return  s + ',' + Float.toString(aX) + ',' + Float.toString(aY) + ',' 
				+ Float.toString(aZ) + ',' + Float.toString(rX) + ',' 
				+ Float.toString(rY) + ',' + Float.toString(rZ) + ',' 
				+ Float.toString(mX) + ',' + Float.toString(mY) + ',' 
				+ Float.toString(mZ);
	}

	@Override
	public void fromLogString(String str) {
		// TODO Auto-generated method stub
		
	}
}
