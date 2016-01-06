package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * Message representing a camera measurement
 * @author ?
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class CameraMeasurement extends BaseMessage implements Message {
	private static final String VERSION_ID = "camV0.0";

	private Date timestamp;

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return String.format("%s,%s\n", formatDate(timestamp),
				String.valueOf(5));
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = tryToParseDate(spl[0]);
		//TODO: actually send a message that is usefull
		return new CameraMeasurement();
	}
}