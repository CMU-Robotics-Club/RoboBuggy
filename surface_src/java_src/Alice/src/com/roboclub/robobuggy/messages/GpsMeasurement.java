package com.roboclub.robobuggy.messages;

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
public class GpsMeasurement implements Message {

	public Date timestamp;

	public float latt;
	public boolean north;
	public float longt;
	public boolean west;

	public GpsMeasurement(float latitude, float longitude) {
		latt = latitude;
		longt = longitude;
	}

	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void fromLogString(String str) {
		// TODO Auto-generated method stub

	}

}