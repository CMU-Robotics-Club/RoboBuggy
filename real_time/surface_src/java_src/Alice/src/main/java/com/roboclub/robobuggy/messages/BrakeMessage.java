package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * Message used for representing the state of the brakes
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class BrakeMessage extends BaseMessage implements Message {

	private Date timestamp;
	private boolean down;
	private static final String VERSION_ID = "brakeV0.1";

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
		this.timestamp = new Date();
	}

	/**
	 * Construct a new {@link BrakeMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param brakeValue the current value of the brakes
	 */
	public BrakeMessage(Date timestamp, boolean brakeValue) {
		this.down = brakeValue;
		this.timestamp = new Date(timestamp.getTime());
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return String.format("%s,'%s',%s", formatDate(timestamp),
				VERSION_ID, String.valueOf(down));
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = tryToParseDate(spl[0]);
		boolean brakeState = Boolean.parseBoolean(spl[2]);
		return new BrakeMessage(d, brakeState);
	}
}