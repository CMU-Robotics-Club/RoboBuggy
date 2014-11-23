package com.roboclub.robobuggy.ros;

public enum CommandChannel {
	STEERING("steering"),
	BRAKE("brake");
	
	private String msgPath;
	
	private CommandChannel(String name) {
		this.msgPath = name;
	}
	
	public String getMsgPath() {
		return this.msgPath;
	}
}
