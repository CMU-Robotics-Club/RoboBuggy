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
public class BrakeMessage extends BaseMessage implements Message {

	public Date timestamp;
	public boolean down;
	public static final String version_id = "brakeV0.1";

	// Makes an encoder measurement with the time of Now.
	public BrakeMessage(int brake_value) {
		switch (brake_value) {
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

	public BrakeMessage(Date timestamp, boolean brake_is_down) {
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
		boolean brake_state = Boolean.parseBoolean(spl[2]);
		return new BrakeMessage(d, brake_state);
	}
}