package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

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

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return String.format("%s,'%s',%s", formatDate(timestamp),
				VERSION_ID, String.valueOf(this.timeValue));
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = tryToParseDate(spl[0]);
		int timeValue = Integer.parseInt(spl[2]);
		return new EncoderTimeMessage(d, timeValue);
	}
}