package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message used for representing the up time of the Arduino Mega
 * @version 0.5
 * @author Sean
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 *          
 */
public class MegaTimeMessage extends BaseMessage {

	public static final String VERSION_ID = "MegaTimeV0.1";
	private int timeValue;

	/**
	 * Construct a new {@link MegaTimeMessage} at time now
	 * @param timeValue The reported up-time of the Arduino Mega
	 */
	public MegaTimeMessage(int timeValue) {
		this.timeValue = timeValue;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link MegaTimeMessage}
	 * @param timestamp {@link Date} representing the Surface time of the message
	 * @param timeValue The reported up-time of the Arduino Mega
	 */
	public MegaTimeMessage(Date timestamp, int timeValue) {
		this.timeValue = timeValue;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}
	
	/**
	 * Returns the up-time field of this particular object
	 * @return timeValue
	 */
	public int getTimeValue(){
		return timeValue;
	}

}