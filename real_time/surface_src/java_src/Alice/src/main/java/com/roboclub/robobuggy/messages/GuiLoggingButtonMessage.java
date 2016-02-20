package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message for passing gui logging button status
 * @author ?
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class GuiLoggingButtonMessage extends BaseMessage {

	/**
	 * Logging message state
	 */
	public enum LoggingMessage {
		START, STOP		
	}
	
	public static final String VERSION_ID = "gui_logging_buttonV0.0";

	private LoggingMessage lm;
	
	/**
	 * Constructs a new {@link GuiLoggingButtonMessage} at time now
	 * @param lm {@link LoggingMessage} state of the GUI
	 */
	public GuiLoggingButtonMessage(LoggingMessage lm) {
		this.lm = lm;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Constructs a new {@link GuiLoggingButtonMessage} at time now
	 * @param timestamp {@link Date} representing the time of the message
	 * @param lm {@link LoggingMessage} state of the GUI
	 */
	public GuiLoggingButtonMessage(Date timestamp, LoggingMessage lm) {
		this.timestamp = new Date(timestamp.getTime()).getTime();
		this.lm = lm;
	}

	/**
	 * @return the logging message
	 */
	public LoggingMessage getLoggingMessage() {
		return lm;
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		String s = formatDate(timestamp);
		return s + ',' + lm.toString();
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] ar = str.split(",");
		Date d = tryToParseDate(ar[0]);
		LoggingMessage lm = LoggingMessage.valueOf(ar[1]);
		return new GuiLoggingButtonMessage(d, lm);
	}
}