package com.roboclub.robobuggy.messages;

import java.util.Date;
/**
 * Message sent by low level indicating the brake command from the RC controller
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */


public class TeleopBrakeStateMessage extends BaseMessage {

	public static final String VERSION_ID = "teleopbrakeV0.1";
	private boolean down;

	/**
	 * Construct a new {@link TeleopBrakeStateMessage} at time now
	 * @param isDown the commanded state of the by the RC controller
	 */
	public TeleopBrakeStateMessage(boolean isDown) {
		this.down = isDown;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link TeleopBrakeStateMessage} at time now
	 * @param brakeValue the current value of the brakes
	 */
	public TeleopBrakeStateMessage(int brakeValue) {
		switch (brakeValue) {
		case 0:
			this.down = false;
			break;
		case 1:
			this.down = true;
			break;
		default:
			this.down = false;
		}
		this.timestamp = new Date().getTime();
	}
	
	
	/**
	 * Construct a new {@link TeleopBrakeStateMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param isDown the current value of the brakes
	 */
	public TeleopBrakeStateMessage(Date timestamp, boolean isDown) {
		this.down = isDown;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}
	
	/**
	 * getState
	 * 
	 * @return The commanded brake state according to low level
	 */
	public boolean getState(){
		return this.down;
	}

	/**
	 * getTime
	 * @return The time the message was received
	 */
	public long getTime(){
		return this.timestamp;
	}
	
}