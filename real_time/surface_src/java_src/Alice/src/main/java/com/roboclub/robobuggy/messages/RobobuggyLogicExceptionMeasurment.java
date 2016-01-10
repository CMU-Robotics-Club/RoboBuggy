package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.ros.Message;

public class RobobuggyLogicExceptionMeasurment extends BaseMessage implements Message{
	private String message;
	private MessageLevel level;
	
	public RobobuggyLogicExceptionMeasurment(String message,MessageLevel level) {
		this.message = message;
		this.level = level;
	}

	
	@Override
	public String toLogString() {
		return level.toString() + "\t"+message;
	}
	
	public String getMessage(){
		return message;	
	}
	
	public MessageLevel getLevel(){
		return level;
	}

	@Override
	public Message fromLogString(String str) {
		int splitPoint = str.indexOf("\t");
		if(splitPoint <0 ){
			//error 
			return null;
		}else{
			String thisLevel_str = str.substring(0, splitPoint);
			MessageLevel thisLevel = null;
			if(thisLevel_str.equals(MessageLevel.EXCEPTION.toString())){
				thisLevel = MessageLevel.EXCEPTION;
			}else if(thisLevel_str.equals(MessageLevel.NOTE.toString())){
				thisLevel = MessageLevel.NOTE;
			}else if(thisLevel_str.equals(MessageLevel.WARNING.toString())){
				thisLevel = MessageLevel.WARNING;
			}
			String thisMessage = str.substring(splitPoint, str.length());
			return new RobobuggyLogicExceptionMeasurment(thisMessage, thisLevel);
		}
	}

}
