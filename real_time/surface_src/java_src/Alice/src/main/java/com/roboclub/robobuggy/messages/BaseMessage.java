package com.roboclub.robobuggy.messages;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.roboclub.robobuggy.main.MESSAGE_LEVEL;
import com.roboclub.robobuggy.main.RobobuggyLogicException;

public abstract class BaseMessage {
	public static DateFormat formatter = new SimpleDateFormat(
			"yyyy-MM-dd HH:mm:ss.SSS");

	public static String format_the_date(Date dt) {
		return formatter.format(dt);
	}

	public static Date try_to_parse_date(String maybe_date) {
		try {
			return formatter.parse(maybe_date);
		} catch (ParseException e) {
			new RobobuggyLogicException("could not parse date stack trace: "+ e.getStackTrace().toString(), MESSAGE_LEVEL.WARNING);
			return null;
		}
	}
}
