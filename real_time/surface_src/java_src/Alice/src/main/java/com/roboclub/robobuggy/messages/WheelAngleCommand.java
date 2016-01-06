package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * Message used to pass commanded steering angle messages over BuggyROS
 * @author ?
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class WheelAngleCommand implements Message {
	// V0.0 had int
	// V0.1 has float. Note that round-off is hundreths of a degree.
	public static final String VERSION_ID = "autonomous_angleV0.1";

	private Date timestamp;
	private float angle;

	/**
	 * Construct a new {@link WheelAngleCommand} object at time now
	 * @param angle the commanded angle 
	 * (Note that round-off is hundreths of a degree)
	 */
	public WheelAngleCommand(float angle) {
		this.angle = angle;
		this.timestamp = new Date();
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		// TODO Auto-generated method stub
		return null;

	}

}