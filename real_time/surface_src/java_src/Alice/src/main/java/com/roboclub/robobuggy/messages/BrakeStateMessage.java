package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

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
	 * Construct a new {@link BatteryMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param stateValue  the current value of the brakes according to low level
	 */
	public BrakeStateMessage(Date timestamp, int stateValue) {
		this.state = stateValue;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return String.format("%s,'%s',%s", formatDate(timestamp),
				VERSION_ID, String.valueOf(this.state));
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = tryToParseDate(spl[0]);
		int stateValue = Integer.parseInt(spl[2]);
		return new BatteryLevelMessage(d, stateValue);
	}
}