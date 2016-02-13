package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message for sending a reset command over BuggyROS
 */
public class ResetMessage extends BaseMessage {
	public static final String VERSION_ID = "reset";

	/**
	 * Constructs a new {@link ResetMessage}
	 */
	public ResetMessage() {
		this.timestamp = new Date().getTime();
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
