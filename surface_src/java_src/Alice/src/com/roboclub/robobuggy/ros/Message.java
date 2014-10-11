package com.roboclub.robobuggy.ros;


public interface Message {
	// TODO force messages to have a time
	
	public String toLogString();
	public String fromLogString(String str);
}
