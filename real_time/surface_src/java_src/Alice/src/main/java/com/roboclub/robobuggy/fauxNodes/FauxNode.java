package com.roboclub.robobuggy.fauxNodes;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;


public class FauxNode {

	BlockingQueue<String> q;
	boolean parserStarted = false;
	static long offset = 0;
	public String typeString;
	private boolean enabled = true;
	
	public FauxNode() {
		q = new ArrayBlockingQueue<String>(50);
		typeString = "";
	}
	
//	@Override
	public void parse() {
		return;
		// TODO Auto-generated method stub
	}

//	@Override
	public void qAdd(String input) {
		if (enabled) {
			while (true) {
				if (q.offer(input)) {
					break;
				}
			}
			if (!parserStarted) {
				parse();
				parserStarted = true;
			}
			return;
		}
	}
	
	public void enable() {
		this.enabled = true;
	}
	
	public void disable() {
		this.enabled = false;
	}
	
	public static void setOffset(long offset) {
		FauxNode.offset = offset;		
	}
	
	protected void sleep(Date target) {
		try {
			long sleepTime = target.getTime() + offset - new Date().getTime();
			if (sleepTime > 0) {
				Thread.sleep(sleepTime);
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
		
	public static Date makeDate(String string) {
		try {
			DateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.ENGLISH);
			Date date = format.parse(string);
			return date;
		} catch (ParseException e) {
			System.out.println("Unable to parse date");
			return null;
		}
	}
	
	public static String makeStringFromDate(Date date) {
		try {
			DateFormat formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.ENGLISH);
			String ret = formatter.format(date);
			return ret;
		} catch (Exception e) {
			System.out.println("Unable to convert date to string properly");
			return null;
		}
	}

	//Adjusts the start time to current start, which is start of program
	//Answer in milliseconds
	public static long calculateOffset(Date current, Date start) {
		return current.getTime() - start.getTime();
	}
}