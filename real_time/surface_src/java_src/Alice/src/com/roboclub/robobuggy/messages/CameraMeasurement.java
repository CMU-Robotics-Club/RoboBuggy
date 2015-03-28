package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

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
		boolean brake_state = Boolean.parseBoolean(spl[1]);
		return new BrakeMessage(d, brake_state);
	}
}