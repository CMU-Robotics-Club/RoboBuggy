package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

public class FingerPrintMessage extends BaseMessage implements Message
{
	public int fpHash;
	public Date timestamp;
	
	public FingerPrintMessage(int hash)
	{
		this.fpHash = hash;
		this.timestamp = new Date();
	}
	
	public FingerPrintMessage(Date tStamp, int hash)
	{
		this.fpHash = hash;
		this.timestamp = tStamp;
	}
	
	@Override
	public String toLogString() {
		return super.formatter.format(timestamp) + ',' 
				+ Integer.toString(this.fpHash);
	}

	@Override
	public Message fromLogString(String str) {
		String[] ar = str.split(",");
		Date d = try_to_parse_date(ar[0]);
		int hash = Integer.parseInt(ar[1]);
		return new FingerPrintMessage(d, hash);
	}

}
