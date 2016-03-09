package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message used for representing the ID of the sending device
 * @version 0.5
 * @author Sean
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 *          
 */
public class DeviceIDMessage extends BaseMessage {

	public static final String VERSION_ID = "IDV0.1";
	private int id;

	/**
	 * Construct a new {@link DeviceIDMessage} at time now
	 * @param idValue the id of the sending device
	 */
	public DeviceIDMessage(int idValue) {
		this.id = idValue;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link DeviceIDMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param idValue the current value of the brakes
	 */
	public DeviceIDMessage(Date timestamp, int idValue) {
		this.id = idValue;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}
	
	/**
	 * evaluates to the device id
	 * @return the device id
	 */
	public int getID(){
		return id;
	}

}