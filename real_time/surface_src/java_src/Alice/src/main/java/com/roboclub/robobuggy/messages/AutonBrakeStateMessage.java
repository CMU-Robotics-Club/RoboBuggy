package com.roboclub.robobuggy.messages;

import java.util.Date;


/**
 * Message sent by low level indicating the brake command received from high level
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class AutonBrakeStateMessage extends BaseMessage {

	public static final String VERSION_ID = "autonbrakeV0.1";
	private boolean down;

	/**
	 * Construct a new {@link AutonBrakeStateMessage} at time now
	 * @param brakeValue the current value of the brakes
	 */
	public AutonBrakeStateMessage(int brakeValue) {
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
	 * Construct a new {@link AutonBrakeStateMessage} at time now
	 * @param isDown the current value of the brakes
	 */
	public AutonBrakeStateMessage(boolean isDown) {
		this.down = isDown;
		this.timestamp = new Date().getTime();
	}
	

	/**
	 * Construct a new {@link AutonBrakeStateMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param brakeValue the current value of the brakes
	 */
	public AutonBrakeStateMessage(Date timestamp, boolean brakeValue) {
		this.down = brakeValue;
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