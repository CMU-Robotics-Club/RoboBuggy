package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * Message for sending a reset command over BuggyROS
 */
public class ResetMessage implements Message {
	private Date timestamp;
	
	/**
	 * Constructs a new {@link ResetMessage}
	 */
	public ResetMessage() {
		this.timestamp = new Date();
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		// TODO Auto-generated method stub
		return null;
		
	}
}
