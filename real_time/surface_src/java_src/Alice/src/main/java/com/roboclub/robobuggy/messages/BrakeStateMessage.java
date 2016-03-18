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
	private int state;

	/**
	 * Construct a new {@link BrakeStateMessage} at time now
	 * @param stateValue the current value of the brakes according to low level
	 */
	public BrakeStateMessage(int stateValue) {
		this.state = stateValue;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link BrakeStateMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param stateValue  the current value of the brakes according to low level
	 */
	public BrakeStateMessage(Date timestamp, int stateValue) {
		this.state = stateValue;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}
	
	/**
	 * evaluates to the brake state
	 * @return brakestate
	 */
	public int getBrakeState(){
		return  state;
	}

}