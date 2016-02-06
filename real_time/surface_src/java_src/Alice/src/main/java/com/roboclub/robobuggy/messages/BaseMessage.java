package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Abstract class used to represent the base message sent over BuggyROS
 */
public abstract class BaseMessage implements Message {
	private static final String DATE_FORMAT = "yyyy-MM-dd HH:mm:ss.SSS";
	protected Date timestamp;

	/**
	 * sets the date of the message to the current time
	 */
	public BaseMessage() {
		timestamp = new Date();
	}

	/**
	 * Creates a {@link String} representing the {@link Date}
	 * @param dt {@link Date} to format
	 * @return a {@link String} representing the {@link Date} dt
	 */
	public static String formatDate(Date dt) {
		DateFormat formatter = new SimpleDateFormat(DATE_FORMAT);
		return formatter.format(dt);
	}

	/**
	 * Converts a {@link String} into a valid {@link Date} object, if possible
	 * @param maybeDate {@link String} representing the {@link Date}
	 * @return a new {@link Date} object representing maybeDate, or null if
	 *  maybeDate is an invalid format
	 */
	public static Date tryToParseDate(String maybeDate) {
		try {
			DateFormat formatter = new SimpleDateFormat(DATE_FORMAT);
			return formatter.parse(maybeDate);
		} catch (ParseException e) {
			new RobobuggyLogicNotification("could not parse date stack trace: "+ e.getMessage(), RobobuggyMessageLevel.WARNING);
			return null;
		}
	}

	/**
	 * Returns the timestamp of the message
	 * @return time that this message was instantiated
	 */
	public Date getTimestamp() {
		return new Date(timestamp.getTime());
	}
}
