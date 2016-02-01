package com.roboclub.robobuggy.utilities;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Class used to format dates
 */
public class RobobuggyDateFormatter {

	/**
	 * Converts a {@link String} of a date into a {@link Date} object
	 * @param date {@link String} representation of the date
	 * @return {@link Date} object representing the date
	 */
	public static Date formatRobobuggyDate(String date) {
		try {
			if(date == null){
				new RobobuggyLogicNotification("No Date information sent to formatRobobuggyDate", RobobuggyMessageLevel.EXCEPTION);
				return new  Date();
			}else{
				DateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.ENGLISH);
				return format.parse(date);
			}
		} catch (ParseException e) {
			System.out.println("Unable to parse date");
			new RobobuggyLogicNotification("Couldn't parse date from " + date, RobobuggyMessageLevel.EXCEPTION);
			return new Date();
		}	
	}
	
}
	
