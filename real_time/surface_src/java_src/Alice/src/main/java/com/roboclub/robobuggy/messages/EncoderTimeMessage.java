package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message used for representing the timestamp of the encoder
 * @version 0.5
 * @author Sean
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 *          
 */
public class EncoderTimeMessage extends BaseMessage {

	public static final String VERSION_ID = "EncoderTimeV0.1";
	private int timeValue;

	/**
	 * Construct a new {@link EncoderTimeMessage} at time now
	 * @param timeValue the current value of the brakes
	 */
	public EncoderTimeMessage(int timeValue) {
		this.timeValue = timeValue;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link EncoderTimeMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param timeValue the current value of the brakes
	 */
	public EncoderTimeMessage(Date timestamp, int timeValue) {
		this.timeValue = timeValue;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}
	
	/**
	 * evaluates to the timeValue of the encoder
	 * @return timeValue
	 */
	public int getTimeValue(){
		return timeValue;
	}

}