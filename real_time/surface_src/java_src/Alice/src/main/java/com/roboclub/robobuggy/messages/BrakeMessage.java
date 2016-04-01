package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message used for representing the state of the brakes
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class BrakeMessage extends BaseMessage {

	public static final String VERSION_ID = "brakeV0.1";
	private boolean down;

	/**
	 * Construct a new {@link BrakeMessage} at time now
	 * @param brakeValue the current value of the brakes
	 */
	public BrakeMessage(int brakeValue) {
		switch (brakeValue) {
		case 0:
			down = false;
			break;
		case 1:
			down = true;
			break;
		default:
			down = false;
		}
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link BrakeMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param brakeValue the current value of the brakes
	 */
	public BrakeMessage(Date timestamp, boolean brakeValue) {
		this.down = brakeValue;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}

	public boolean isDown() {
		return down;
	}

	public void setDown(boolean down) {
		this.down = down;
	}
}