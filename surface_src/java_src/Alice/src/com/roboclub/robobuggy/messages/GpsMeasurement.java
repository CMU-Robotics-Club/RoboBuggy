package com.roboclub.robobuggy.messages;

import java.util.Date;

// Represents raw measurement from the IMU
public class GpsMeasurement {
	
	public Date timestamp;

	public float latt;
	public boolean north;
	public float longt;
	public boolean west;

}