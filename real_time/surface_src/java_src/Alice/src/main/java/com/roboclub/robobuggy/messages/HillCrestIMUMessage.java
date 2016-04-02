package com.roboclub.robobuggy.messages;
/**
 *  A temp message for storing the hilcrest imu's raw data 
 * @author Trevor Decker
 *
 */
public class HillCrestIMUMessage  extends BaseMessage{
	private String message ="";
	
	public static final String VERSION_ID = "hillcrest_imuV0.0";
	
	/**
	 * constructor for the datatype 
	 * @param message message to encode 
	 */
	public HillCrestIMUMessage(String message) {
		this.message = message;
		// TODO Auto-generated constructor stub
	}
	
	/**
	 * evaluates to a string encoding of the message
	 * @return string encoding of the message
	 */
	public String getMessage(){
		return message;
	}
	
}
