package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * Message for passing the desired commanded value of the brakes
 */
public class BrakeControlMessage extends BaseMessage implements Message {

	private static final String VERSION_ID = "brake_control_message";
	
	private final boolean brakeEngaged;
	private final Date timestamp;
	
	/**
	 * Construct a new BrakeControlMessage
	 * @param timestamp {@link Date} representing the creation time
	 * @param brakeEngagged whether or not the brakes are deployed
	 */
	public BrakeControlMessage(Date timestamp, boolean brakeEngagged) {
		this.brakeEngaged = brakeEngagged;
		this.timestamp = new Date(timestamp.getTime());
	}
	
	/**
	 * Returns true iff the brake is commanded to be engaged
	 * @return true iff the brake is commanded to be engaged
	 */
	public boolean isBrakeEngaged() {
		return brakeEngaged;
	}
	
	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return String.format("%s,'%s',%s", formatDate(timestamp),
				VERSION_ID, String.valueOf(brakeEngaged));
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = tryToParseDate(spl[0]);
		boolean brakeState = Boolean.parseBoolean(spl[2]);
		return new BrakeControlMessage(d, brakeState);
	}

}
