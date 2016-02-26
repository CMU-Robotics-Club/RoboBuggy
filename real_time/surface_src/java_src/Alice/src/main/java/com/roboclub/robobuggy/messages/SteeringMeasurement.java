package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message for passing steering angle measurements over BuggyROS
 */
public class SteeringMeasurement extends BaseMessage {
	public static final String VERSION_ID = "steering";
	private int angle;

	/**
	 * Construct a new {@link SteeringMeasurement} at time now
	 * @param angle front wheel angle
	 */
	public SteeringMeasurement(int angle) {
		this.timestamp = new Date().getTime();
		this.angle = angle;
	}
	
	/**
	 * Returns the angle of the {@link SteeringMeasurement}
	 * @return the angle of the {@link SteeringMeasurement}
	 */
	public int getAngle() {
		return angle;
	}
	
	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		String s = formatDate(timestamp);
		return s + ',' + Double.toString(angle);
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String delims = ",";
		String[] ar = str.split(delims);
		timestamp = new Date(tryToParseDate(ar[0]).getTime()).getTime();
		int angle = Integer.parseInt(ar[1]);
		return new SteeringMeasurement(angle);
	}
}
