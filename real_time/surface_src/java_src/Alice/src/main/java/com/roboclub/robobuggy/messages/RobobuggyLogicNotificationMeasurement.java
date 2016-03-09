package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

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

}

