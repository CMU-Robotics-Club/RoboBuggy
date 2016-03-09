package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message used for representing the auton state reported by low level
 * @version 0.5
 * @author Sean
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 *          
 */
public class AutonStateMessage extends BaseMessage {

	public static final String VERSION_ID = "AUTONSTATEV0.1";
	private int state;

	/**
	 * Construct a new {@link AutonStateMessage} at time now
	 * @param stateValue the reported auton state
	 */
	public AutonStateMessage(int stateValue) {
		this.state = stateValue;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link AutonStateMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param stateValue the current value of auton
	 */
	public AutonStateMessage(Date timestamp, int stateValue) {
		this.state = stateValue;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}
	
	/**
	 * Evaluates to the current state 
	 * @return state 
	 */
	public int getState(){
		return this.state;
	}

}