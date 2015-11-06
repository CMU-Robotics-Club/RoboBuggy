package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.MESSAGE_LEVEL;
import com.roboclub.robobuggy.ros.Message;

public class RobobuggyLogicExceptionMeasurment extends BaseMessage implements Message{
	private String message;
	private MESSAGE_LEVEL level;
	
	public RobobuggyLogicExceptionMeasurment(String message,MESSAGE_LEVEL level) {
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
			if(thisLevel_str.equals(MESSAGE_LEVEL.EXCEPTION.toString())){
				thisLevel = MESSAGE_LEVEL.EXCEPTION;
			}else if(thisLevel_str.equals(MESSAGE_LEVEL.NOTE.toString())){
				thisLevel = MESSAGE_LEVEL.NOTE;
			}else if(thisLevel_str.equals(MESSAGE_LEVEL.WARNING.toString())){
				thisLevel = MESSAGE_LEVEL.WARNING;
			}
			String thisMessage = str.substring(splitPoint, str.length());
			return new RobobuggyLogicExceptionMeasurment(thisMessage, thisLevel);
		}
	}

}
