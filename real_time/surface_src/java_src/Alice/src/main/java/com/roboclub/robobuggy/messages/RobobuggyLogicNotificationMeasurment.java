package com.roboclub.robobuggy.messages;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;

import java.util.Date;

/**
 * Message for passing logic exceptions over BuggyRos
 */
public class RobobuggyLogicNotificationMeasurment extends BaseMessage implements Message{
	private String message;
	private RobobuggyMessageLevel level;
	private Date timestamp;
	
	/**
	 * Constructs a new RobobuggyLogicExceptionMeasurement at time now.
	 * @param message {@link String} message to transmit
	 * @param level {@link RobobuggyMessageLevel} of the message to transmit
	 */
	public RobobuggyLogicNotificationMeasurment(String message, RobobuggyMessageLevel level) {
		this.message = message;
		this.level = level;
		this.timestamp = new Date();
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return formatDate(timestamp) + "," + level.toString() + "," + message;
	}
	
	/**
	 * Returns the {@link String} representing the contents of the {@link RobobuggyLogicNotification}
	 * @return the {@link String} representing the contents of the {@link RobobuggyLogicNotification}
	 */
	public String getMessage(){
		return message;	
	}

	/**
	 * Returns the {@link RobobuggyMessageLevel} of the {@link com.roboclub.robobuggy.main.RobobuggyLogicNotification }
	 * @return the {@link RobobuggyMessageLevel} of the {@link RobobuggyLogicNotification}
	 */
	public RobobuggyMessageLevel getLevel(){
		return level;
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		int splitPoint = str.indexOf("\t");
		if(splitPoint < 0){
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
			return new RobobuggyLogicNotificationMeasurment(thisMessage, thisLevel);
		}
	}

	/**
	 * Translates a peel message into a JsonObject for logging
	 * @param message the raw line from the subscriber
	 * @return the filed Json object
	 */
	@SuppressWarnings("unchecked")
	public static JSONObject translatePeelMessageToJObject(String message) {
		JSONObject data = new JSONObject();
		JSONObject params = new JSONObject();
		String[] messageData = message.split(",");
		data.put("timestamp", messageData[1]);
		params.put("level", messageData[2]);
		params.put("message", messageData[3]);

		data.put("params", params);
		return data;
	}


}
