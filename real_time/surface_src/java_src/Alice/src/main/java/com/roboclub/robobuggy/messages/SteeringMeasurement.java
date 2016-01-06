package com.roboclub.robobuggy.messages;

import java.util.Date;
import com.roboclub.robobuggy.ros.Message;

/**
 * Message for passing steering angle measurements over BuggyROS
 */
public class SteeringMeasurement extends BaseMessage implements Message {
	private int angle;
	private Date timestamp;
	
	/**
	 * Construct a new {@link SteeringMeasurement} at time now
	 * @param angle front wheel angle
	 */
	public SteeringMeasurement(int angle) {
		this.timestamp = new Date();
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
		Date timestamp = tryToParseDate(ar[0]);
		int angle = Integer.parseInt(ar[1]);
		return new SteeringMeasurement(angle);
	}
}
