package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.MESSAGE_LEVEL;
import com.roboclub.robobuggy.ros.Message;

public class LogicExceptionMeasurment extends BaseMessage implements Message{
	String message;
	MESSAGE_LEVEL level;
	
	public LogicExceptionMeasurment(String message,MESSAGE_LEVEL level) {
		this.message = message;
		this.level = level;
	}

	
	@Override
	public String toLogString() {
		return level.toString() + "\t"+message;
	}

	@Override
	public Message fromLogString(String str) {
		int splitPoint = str.indexOf("\t");
		if(splitPoint <0 ){
			//error 
			return null;
		}else{
			String thisLevel_str = str.substring(0, splitPoint);
			MESSAGE_LEVEL thisLevel = null;
			if(thisLevel_str.equals(MESSAGE_LEVEL.exception.toString())){
				thisLevel = MESSAGE_LEVEL.exception;
			}else if(thisLevel_str.equals(MESSAGE_LEVEL.note.toString())){
				thisLevel = MESSAGE_LEVEL.note;
			}else if(thisLevel_str.equals(MESSAGE_LEVEL.warning.toString())){
				thisLevel = MESSAGE_LEVEL.warning;
			}
			String thisMessage = str.substring(splitPoint, str.length());
			return new LogicExceptionMeasurment(thisMessage, thisLevel);
		}
	}

}
