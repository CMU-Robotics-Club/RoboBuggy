package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.ros.Message;

/**
 * Message for passing logic exceptions over BuggyRos
 */
public class RobobuggyLogicExceptionMeasurment extends BaseMessage implements Message{
	private String message;
	private MessageLevel level;
	
	/**
	 * Constructs a new {@link RobobuggyLogicExceptionMeasurement} at time now.
	 * @param message {@link String} message to transmit
	 * @param level {@link MessageLevel} of the message to transmit
	 */
	public RobobuggyLogicExceptionMeasurment(String message,MessageLevel level) {
		this.message = message;
		this.level = level;
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return level.toString() + "\t"+message;
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		int splitPoint = str.indexOf("\t");
		if(splitPoint <0 ){
			//error 
			return null;
		}else{
			String thisLevelStr = str.substring(0, splitPoint);
			MessageLevel thisLevel = null;
			if(thisLevelStr.equals(MessageLevel.EXCEPTION.toString())){
				thisLevel = MessageLevel.EXCEPTION;
			}else if(thisLevelStr.equals(MessageLevel.NOTE.toString())){
				thisLevel = MessageLevel.NOTE;
			}else if(thisLevelStr.equals(MessageLevel.WARNING.toString())){
				thisLevel = MessageLevel.WARNING;
			}
			String thisMessage = str.substring(splitPoint, str.length());
			return new RobobuggyLogicExceptionMeasurment(thisMessage, thisLevel);
		}
	}

}
