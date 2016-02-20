package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message for passing logic exceptions over BuggyRos
 */
public class RobobuggyLogicNotificationMeasurement extends BaseMessage {
	public static final String VERSION_ID = "logic_notification";
	private String message;
	private RobobuggyMessageLevel level;

	/**
	 * Constructs a new RobobuggyLogicExceptionMeasurement at time now.
	 *
	 * @param message {@link String} message to transmit
	 * @param level   {@link RobobuggyMessageLevel} of the message to transmit
	 */
	public RobobuggyLogicNotificationMeasurement(String message, RobobuggyMessageLevel level) {
		this.message = message;
		this.level = level;
		this.timestamp = new Date().getTime();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toLogString() {
		return formatDate(timestamp) + "," + level.toString() + "," + message;
	}

	/**
	 * Returns the {@link String} representing the contents of the {@link RobobuggyLogicNotification}
	 *
	 * @return the {@link String} representing the contents of the {@link RobobuggyLogicNotification}
	 */
	public String getMessage() {
		return message;
	}

	/**
	 * Returns the {@link RobobuggyMessageLevel} of the {@link com.roboclub.robobuggy.main.RobobuggyLogicNotification }
	 *
	 * @return the {@link RobobuggyMessageLevel} of the {@link RobobuggyLogicNotification}
	 */
	public RobobuggyMessageLevel getLevel() {
		return level;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Message fromLogString(String str) {
		int splitPoint = str.indexOf("\t");
		if (splitPoint < 0) {
			//error
			return null;
		} else {
			String thisLevelStr = str.substring(0, splitPoint);
			RobobuggyMessageLevel thisLevel = null;
			if (thisLevelStr.equals(RobobuggyMessageLevel.EXCEPTION.toString())) {
				thisLevel = RobobuggyMessageLevel.EXCEPTION;
			} else if (thisLevelStr.equals(RobobuggyMessageLevel.NOTE.toString())) {
				thisLevel = RobobuggyMessageLevel.NOTE;
			} else if (thisLevelStr.equals(RobobuggyMessageLevel.WARNING.toString())) {
				thisLevel = RobobuggyMessageLevel.WARNING;
			}
			String thisMessage = str.substring(splitPoint, str.length());
			return new RobobuggyLogicNotificationMeasurement(thisMessage, thisLevel);
		}
	}
}

