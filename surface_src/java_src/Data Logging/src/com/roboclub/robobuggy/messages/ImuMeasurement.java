package com.roboclub.robobuggy.messages;

import java.util.Date;

// Represents raw measurement from the IMU
public class ImuMeasurement {
	
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
}
