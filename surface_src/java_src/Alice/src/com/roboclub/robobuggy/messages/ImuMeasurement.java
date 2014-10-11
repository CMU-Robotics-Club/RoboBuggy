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

	public ImuMeasurement(float aX, float aY, float aZ, 
						float rX, float rY, float rZ, 
						float mX, float mY, float mZ) {
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
		String delims = ",";
		String[] ar = str.split(delims);
		timestamp = (Date) ;
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
