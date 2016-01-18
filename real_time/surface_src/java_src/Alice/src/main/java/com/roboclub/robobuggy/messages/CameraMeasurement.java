package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * @author ?
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

// Represents raw measurement from the IMU
public class CameraMeasurement extends BaseMessage implements Message {
	public static final String version_id = "camV0.0";

	public Date timestamp;

	@Override
	public String toLogString() {
		return String.format("%s,%s\n", format_the_date(timestamp),
				String.valueOf(5));
	}

	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = try_to_parse_date(spl[0]);
		//todo actually send a message that is usefull
		return new CameraMeasurement();
	}
}