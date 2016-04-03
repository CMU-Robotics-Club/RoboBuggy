package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Class used to represent a message containing the low level code fingerprint
 */
public class FingerPrintMessage extends BaseMessage {
	public static final String VERSION_ID = "fingerprint_hash";
	private int fpHash;

	/**
	 * Construct a new {@link FingerPrintMessage} at time now
	 * @param hash the hash of the low level code
	 */
	public FingerPrintMessage(int hash)
	{
		this.fpHash = hash;
		this.timestamp = new Date().getTime();
	}
	
	/**
	 * Construct a new {@link FingerPrintMessage}
	 * @param tStamp {@link Date} representing the time of the message
	 * @param hash the hash of the low level code
	 */
	public FingerPrintMessage(Date tStamp, int hash)
	{
		this.fpHash = hash;
		this.timestamp = new Date(tStamp.getTime()).getTime();
	}

	public int getFpHash() {
		return fpHash;
	}
}
