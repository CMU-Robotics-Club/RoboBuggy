package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;

/**
 * Message for passing logic exceptions over BuggyRos
 */
public class RobobuggyLogicExceptionMeasurment extends BaseMessage implements Message{
	private String message;
	private RobobuggyMessageLevel level;
	
	/**
	 * Constructs a new {@link RobobuggyLogicExceptionMeasurement} at time now.
	 * @param message {@link String} message to transmit
	 * @param level {@link MessageLevel} of the message to transmit
	 */
	public RobobuggyLogicExceptionMeasurment(String message,RobobuggyMessageLevel level) {
		this.message = message;
		this.level = level;
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return level.toString() + "\t"+message;
	}
	
	/**
	 * Returns the {@link String} representing the contents of the {@link RobobuggyLogicException}
	 * @return the {@link String} representing the contents of the {@link RobobuggyLogicException}
	 */
	public String getMessage(){
		return message;	
	}

	/**
	 * Returns the {@link RobobuggyMessageLevel} of the {@link RobobuggyLogicException}
	 * @return the {@link RobobuggyMessageLevel} of the {@link RobobuggyLogicException}
	 */
	public RobobuggyMessageLevel getLevel(){
		return level;
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
			RobobuggyMessageLevel thisLevel = null;
			if(thisLevelStr.equals(RobobuggyMessageLevel.EXCEPTION.toString())){
				thisLevel = RobobuggyMessageLevel.EXCEPTION;
			}else if(thisLevelStr.equals(RobobuggyMessageLevel.NOTE.toString())){
				thisLevel = RobobuggyMessageLevel.NOTE;
			}else if(thisLevelStr.equals(RobobuggyMessageLevel.WARNING.toString())){
				thisLevel = RobobuggyMessageLevel.WARNING;
			}
			String thisMessage = str.substring(splitPoint, str.length());
			return new RobobuggyLogicExceptionMeasurment(thisMessage, thisLevel);
		}
	}

}
