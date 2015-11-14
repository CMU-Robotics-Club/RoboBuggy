package com.roboclub.robobuggy.utilities;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.main.RobobuggyLogicException;

public class RobobuggyDateFormatter {

	public static Date formatRobobuggyDate(String date) {
		try {
			DateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.ENGLISH);
			return format.parse(date);
		} catch (ParseException e) {
			System.out.println("Unable to parse date");
			new RobobuggyLogicException("Couldn't parse date from " + date, MessageLevel.EXCEPTION);
			return new Date();
		}	
	}
	
}
