package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

public class TimeMessage  extends BaseMessage implements Message{
	private long time;
	
	TimeMessage(long newTime){
		time = newTime;
	}
	
	public TimeMessage(Date thisDate){
		time = thisDate.getTime();
	}
	
	public long getTime(){
		return time;
	}
	
	public Date getDate(){
		return new Date(time);
	}
	
	public void setTime(long newTime){
		time = newTime;
	}
	
	public void setTime(Date newTime){
		time = newTime.getTime();
	}
	
	@Override
	public String toLogString() {
		return Long.toString(time);
	}

	@Override
	public Message fromLogString(String str) {
		return  new TimeMessage(Long.parseLong(str));
	}
}
