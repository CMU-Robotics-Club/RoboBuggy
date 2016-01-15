package com.roboclub.robobuggy.utilities;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.RobobuggyLogicException;

public class RobobuggyDateFormatter {

	public static Date formatRobobuggyDate(String date) {
		try {
			if(date == null){
				new RobobuggyLogicException("No Date information sent to formatRobobuggyDate", RobobuggyMessageLevel.EXCEPTION);
				return new  Date();
			}else{
				DateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.ENGLISH);
				return format.parse(date);
			}
		} catch (ParseException e) {
			System.out.println("Unable to parse date");
			new RobobuggyLogicException("Couldn't parse date from " + date, RobobuggyMessageLevel.EXCEPTION);
			return new Date();
		}	
	}
	
}
	