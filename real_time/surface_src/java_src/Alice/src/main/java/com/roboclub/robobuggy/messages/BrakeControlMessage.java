package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * Message for passing the desired commanded value of the brakes
 */
public class BrakeControlMessage extends BaseMessage implements Message {

	private static final String version_id = "brake_control_message";
	
	private final boolean brakeEngaged;
	private final Date timestamp;
	
	/**
	 * Construct a new BrakeControlMessage
	 * @param timestamp
	 * @param brakeEngagged
	 */
	public BrakeControlMessage(Date timestamp, boolean brakeEngagged) {
		this.brakeEngaged = brakeEngagged;
		this.timestamp = timestamp;
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
		return String.format("%s,'%s',%s\n", format_the_date(timestamp),
				version_id, String.valueOf(brakeEngaged));
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = try_to_parse_date(spl[0]);
		boolean brake_state = Boolean.parseBoolean(spl[2]);
		return new BrakeControlMessage(d, brake_state);
	}

}
