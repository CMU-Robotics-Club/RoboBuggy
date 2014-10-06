package com.roboclub.robobuggy.messages;

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
}
