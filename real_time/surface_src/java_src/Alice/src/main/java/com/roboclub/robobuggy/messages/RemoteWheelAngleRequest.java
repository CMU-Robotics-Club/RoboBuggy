package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message to request the front wheel angle
 */
public class RemoteWheelAngleRequest extends BaseMessage {
	public static final String VERSION_ID = "rc_wheel_angleV0.1";

	private double angle;

	/**
	 * Constructs a new {@link RemoteWheelAngleRequest} at time now
	 * @param angle angle of the front wheel
	 */
	public RemoteWheelAngleRequest(double angle) {
		this.angle = angle;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Gets the angle the wheel should be at
	 * @return requested angle
     */
	public double getAngle(){
		return angle;
	}

}