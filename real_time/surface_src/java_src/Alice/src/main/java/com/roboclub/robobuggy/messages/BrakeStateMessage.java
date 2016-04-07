package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message used for representing the brake state reported by low level
 * @version 0.5
 * @author Sean
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 *          
 */
public class BrakeStateMessage extends BaseMessage {

	public static final String VERSION_ID = "BRAKESTATEV0.1";
	private boolean isDown;

	/**
	 * Construct a new {@link BrakeStateMessage} at time now
	 * @param isDown the current value of the brakes according to low level
	 */
	public BrakeStateMessage(boolean isDown) {
		this.isDown = isDown;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link BrakeStateMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param isDown the current value of the brakes according to low level
	 */
	public BrakeStateMessage(Date timestamp, boolean isDown) {
		this.isDown = isDown;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}

	/**
	 * @return whether the brakes are down
	 */
	public boolean isDown() {
		return isDown;
	}

	/**
	 * @param down the state to set the brakes to
	 */
	public void setDown(boolean down) {
		isDown = down;
	}
}