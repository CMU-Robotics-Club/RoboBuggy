package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message for passing the desired commanded value of the steering
 */
public class DriveControlMessage extends BaseMessage {

	public static final String VERSION_ID = "drive_control_message";
	
	private final double angle;

	/**
	 * Construct a new DriveControlMessage
	 * @param timestamp {@link Date} representing the creation time
	 * @param angle the commanded angle of the front wheel
	 */
	public DriveControlMessage(Date timestamp, double angle) {
		this.angle = angle;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}
	
	/**
	 * Returns the commanded angle of the steering as a double (in degrees)
	 * @return the commanded angle of the steering as a double (in degrees)
	 */
	public double getAngleDouble() {
		return angle;
	}
	
	/**
	 * Returns the commanded angle of the steering as an int (in hundredths of degrees)
	 * @return the commanded angle of the steering as an int (in hundredths of degrees)
	 */
	public int getAngleInt() {
		return (int)(angle*100.0);
	}
	
	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return String.format("%s,'%s',%s", formatDate(timestamp),
				VERSION_ID, String.valueOf(angle));
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] spl = str.split(",");
		Date d = tryToParseDate(spl[0]);
		double readAngle = Integer.parseInt(spl[2]);
		return new DriveControlMessage(d, readAngle);
	}

}
