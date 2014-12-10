package com.roboclub.robobuggy.ros;

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
