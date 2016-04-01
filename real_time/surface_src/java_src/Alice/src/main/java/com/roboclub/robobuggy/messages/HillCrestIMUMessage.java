package com.roboclub.robobuggy.messages;

public class HillCrestIMUMessage  extends BaseMessage{
	private String message ="";
	
	public static final String VERSION_ID = "hillcrest_imuV0.0";
	
	public HillCrestIMUMessage(String Message) {
		message = Message;
		// TODO Auto-generated constructor stub
	}
	
	public String getMessage(){
		return message;
	}
	
}
