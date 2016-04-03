package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message used for keeping track of when the encoder has been reset
 * @version 0.5
 * @author Sean
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 *          
 */
public class EncoderResetMessage extends BaseMessage {

	public static final String VERSION_ID = "EncoderResetV0.1";

	/**
	 * Construct a new {@link MegaTimeMessage} at time now
	 */
	public EncoderResetMessage() {
		this.timestamp = new Date().getTime();
	}

	/**
	 * Construct a new {@link EncoderResetMessage}
	 * @param timestamp {@link Date} representing the time of the message
	 */
	public EncoderResetMessage(Date timestamp) {
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}

}