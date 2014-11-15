package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

public class ResetMessage implements Message {
	private Date timestamp;
	
	public ResetMessage() {
		this.timestamp = new Date();
	}

	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void fromLogString(String str) {
		// TODO Auto-generated method stub
		
	}
}
