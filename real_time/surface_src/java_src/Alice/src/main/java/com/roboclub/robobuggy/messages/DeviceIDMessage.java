package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message used for representing the ID of the sending device
 * @version 0.5
 * @author Sean
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 *          
 */
public class DeviceIDMessage extends BaseMessage {

	public static final String VERSION_ID = "IDV0.1";
	private int id;

	/**
	 * Construct a new {@link DeviceIDMessage} at time now
	 * @param idValue the id of the sending device
	 */
	public DeviceIDMessage(int idValue) {
		this.id = idValue;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link DeviceIDMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param idValue the current value of the brakes
	 */
	public DeviceIDMessage(Date timestamp, int idValue) {
		this.id = idValue;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return String.format("%s,'%s',%s", formatDate(timestamp),
				VERSION_ID, String.valueOf(this.id));
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = tryToParseDate(spl[0]);
		int idLevel = Integer.parseInt(spl[2]);
		return new DeviceIDMessage(d, idLevel);
	}
}