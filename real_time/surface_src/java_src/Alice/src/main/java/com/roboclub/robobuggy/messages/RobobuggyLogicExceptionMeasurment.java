package com.roboclub.robobuggy.messages;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.utilities.RobobuggyDateFormatter;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

public class RobobuggyLogicExceptionMeasurment extends BaseMessage implements Message{
	private String message;
	private RobobuggyMessageLevel level;
	private Date timestamp;
	
	public RobobuggyLogicExceptionMeasurment(String message,RobobuggyMessageLevel level) {
		this.message = message;
		this.level = level;
		this.timestamp = new Date();
	}

	
	@Override
	public String toLogString() {
		return format_the_date(timestamp) + "," + level.toString() + "," + message;
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

	public static JSONObject translatePeelMessageToJObject(String message) {
		JSONObject data = new JSONObject();
		JSONObject params = new JSONObject();
		String[] messageData = message.split(",");
		data.put("timestamp", messageData[1]);
		data.put("name", "Robobuggy Logic Exception");
		params.put("level", messageData[2]);
		params.put("message", messageData[3]);

		data.put("params", params);
		return data;
	}

}
