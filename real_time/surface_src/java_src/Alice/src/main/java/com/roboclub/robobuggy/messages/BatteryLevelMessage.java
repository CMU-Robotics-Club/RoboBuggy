package com.roboclub.robobuggy.messages;


import java.util.Date;

/**
 * Message used for representing the level of the battery
 * @version 0.5
 * @author Sean
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 *          
 */
public class BatteryLevelMessage extends BaseMessage {

	public static final String VERSION_ID = "BatteryV0.1";
	private int level;

	/**
	 * Construct a new {@link BatteryLevelMessage} at time now
	 * @param batteryValue the current value of the brakes
	 */
	public BatteryLevelMessage(int batteryValue) {
		this.level = batteryValue;
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link BatteryMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param batteryValue the current value of the brakes
	 */
	public BatteryLevelMessage(Date timestamp, int batteryValue) {
		this.level = batteryValue;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}
	
	/**
	 * evaluates to the battery level 
	 * @return batteryLevel
	 */
	public int getBatteryLevel(){
		return this.level;
	}

}