package org.roboclub.robobuggy.ros;

/**
 * 
 * @author Matt Sebek
 * @version 0.0
 *
 */
public enum ActuatorChannel {
	STEERING("steering"),
	BRAKE("brake"),
	TURNSIGNAL("turn_signal");
	
	private String msgPath;
	
	private ActuatorChannel(String name) {
		this.msgPath = name;
	}
	
	public String getMsgPath() {
		return this.msgPath;
	}
}
