package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;

public class RobobuggyLogicExceptionMeasurment extends BaseMessage implements Message{
	private String message;
	private RobobuggyMessageLevel level;
	
	public RobobuggyLogicExceptionMeasurment(String message,RobobuggyMessageLevel level) {
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
	
	public RobobuggyMessageLevel getLevel(){
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
			RobobuggyMessageLevel thisLevel = null;
			if(thisLevel_str.equals(RobobuggyMessageLevel.EXCEPTION.toString())){
				thisLevel = RobobuggyMessageLevel.EXCEPTION;
			}else if(thisLevel_str.equals(RobobuggyMessageLevel.NOTE.toString())){
				thisLevel = RobobuggyMessageLevel.NOTE;
			}else if(thisLevel_str.equals(RobobuggyMessageLevel.WARNING.toString())){
				thisLevel = RobobuggyMessageLevel.WARNING;
			}
			String thisMessage = str.substring(splitPoint, str.length());
			return new RobobuggyLogicExceptionMeasurment(thisMessage, thisLevel);
		}
	}

}
