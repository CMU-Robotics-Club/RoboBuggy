package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.ros.Message;

/**
 * @author ?
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

// Represents raw measurement from the IMU
public class GuiLoggingButtonMessage extends BaseMessage implements Message {

	public enum LoggingMessage {
		START, STOP		
	}
	
	public static final String version_id = "gui_logging_buttonV0.0";

	public Date timestamp;
	public LoggingMessage lm; 
	
	// Makes an encoder measurement with the time of Now.
	public GuiLoggingButtonMessage(LoggingMessage lm) {
		this.lm = lm;
		this.timestamp = new Date();
	}

	public GuiLoggingButtonMessage(Date timestamp, LoggingMessage lm) {
		this.timestamp = timestamp;
		this.lm = lm;
	}

	@Override
	public String toLogString() {
		String s = super.formatter.format(timestamp);
		return s + ',' + lm.toString();
	}

	@Override
	public Message fromLogString(String str) {
		String[] ar = str.split(",");
		Date d = try_to_parse_date(ar[0]);
		LoggingMessage lm = LoggingMessage.valueOf(ar[1]);
		return new GuiLoggingButtonMessage(timestamp, lm);
	}

	@Override
	public String getCorrespondingSensor() {
		// TODO Auto-generated method stub
		return config.SENSOR_NAME_GUI_LOGGING_BUTTON;
	}
}