package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * Message to request the front wheel angle
 */
public class RemoteWheelAngleRequest implements Message {
	public static final String VERSION_ID = "rc_wheel_angleV0.1";

	private Date timestamp;
	private double angle;

	/**
	 * Constructs a new {@link RemoteWheelAngleRequest} at time now
	 * @param angle angle of the front wheel
	 */
	public RemoteWheelAngleRequest(double angle) {
		this.angle = angle;
		this.timestamp = new Date();
	}
	
	public double getAngle(){
		return angle;
	}
	
	public Date getTimeStamp(){
		return new Date(timestamp.getTime());
	}

	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Message fromLogString(String str) {
		// TODO Auto-generated method stub
		return null;
	}

}