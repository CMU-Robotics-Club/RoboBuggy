package com.roboclub.robobuggy.messages;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.main.RobobuggyLogicException;

/**
 * Abstract class used to represent the base message sent over BuggyROS
 */
public abstract class BaseMessage {
	private static final DateFormat FORMATTER = new SimpleDateFormat(
			"yyyy-MM-dd HH:mm:ss.SSS");

	/**
	 * Creates a {@link String} representing the {@link Date}
	 * @param dt {@link Date} to format
	 * @return a {@link String} representing the {@link Date} dt
	 */
	public static String formatDate(Date dt) {
		return FORMATTER.format(dt);
	}

	/**
	 * Converts a {@link String} into a valid {@link Date} object, if possible
	 * @param maybeDate {@link String} representing the {@link Date}
	 * @return a new {@link Date} object representing maybeDate, or null if
	 *  maybeDate is an invalid format
	 */
	public static Date tryToParseDate(String maybeDate) {
		try {
			return FORMATTER.parse(maybeDate);
		} catch (ParseException e) {
			new RobobuggyLogicException("could not parse date stack trace: "+ e.getStackTrace().toString(), MessageLevel.WARNING);
			return null;
		}
	}
}
