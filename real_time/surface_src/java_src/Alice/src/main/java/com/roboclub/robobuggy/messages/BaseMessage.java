package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Abstract class used to represent the base message sent over BuggyROS
 */
public abstract class BaseMessage {
	private static final String DATE_FORMAT = "yyyy-MM-dd HH:mm:ss.SSS";

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
}
