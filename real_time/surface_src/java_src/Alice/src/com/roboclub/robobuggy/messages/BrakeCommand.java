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
public class BrakeCommand extends BaseMessage implements Message {

	public Date timestamp;
	public boolean down;
	public static final String version_id = "brakeV0.1";

	// Makes an encoder measurement with the time of Now.
	public BrakeCommand(boolean brake_is_down) {
		this.down = brake_is_down;
		this.timestamp = new Date();
	}

	public BrakeCommand(Date timestamp, boolean brake_is_down) {
		this.down = brake_is_down;
		this.timestamp = timestamp;
	}

	@Override
	public String toLogString() {
		return String.format("%s,'%s',%s\n", format_the_date(timestamp),
				version_id, String.valueOf(down));
	}

	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = try_to_parse_date(spl[0]);
		boolean brake_state = Boolean.parseBoolean(spl[1]);
		return new BrakeCommand(d, brake_state);
	}
}