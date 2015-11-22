package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * Message for passing the desired commanded value of the steering
 */
public class DriveControlMessage extends BaseMessage implements Message {

	private static final String version_id = "drive_control_message";
	
	private final double angle;
	private final Date timestamp;
	
	/**
	 * Construct a new DriveControlMessage
	 * @param timestamp
	 * @param brakeEngagged
	 */
	public DriveControlMessage(Date timestamp, double angle) {
		this.angle = angle;
		this.timestamp = timestamp;
	}
	
	/**
	 * Returns the commanded angle of the steering as a double (in degrees)
	 * @return the commanded angle of the steering as a double (in degrees)
	 */
	public double getAngleDouble() {
		return angle;
	}
	
	/**
	 * Returns the commanded angle of the steering as a short (in thousandths of degrees)
	 * @return the commanded angle of the steering as a short (in thousandths of degrees)
	 */
	public short getAngleShort() {
		return (short)(angle*1000.0);
	}
	
	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return String.format("%s,'%s',%s\n", format_the_date(timestamp),
				version_id, String.valueOf(angle));
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = try_to_parse_date(spl[0]);
		double readAngle = Short.parseShort(spl[2]);
		return new DriveControlMessage(d, readAngle);
	}

}
