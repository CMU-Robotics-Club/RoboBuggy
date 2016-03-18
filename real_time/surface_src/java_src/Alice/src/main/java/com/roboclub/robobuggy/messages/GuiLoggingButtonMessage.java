package com.roboclub.robobuggy.messages;

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

}