package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

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
	
	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return formatDate(timestamp) + ',' 
				+ Integer.toString(this.fpHash);
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] ar = str.split(",");
		Date d = tryToParseDate(ar[0]);
		int hash = Integer.parseInt(ar[1]);
		return new FingerPrintMessage(d, hash);
	}

}
