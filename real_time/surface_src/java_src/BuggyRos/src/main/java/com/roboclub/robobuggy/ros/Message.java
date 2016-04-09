package com.roboclub.robobuggy.ros;

/**
 * @author Matt Sebek
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

public interface Message {

    public void setTopicName(String name);
    public String getTopicName();
	public long getSequenceNumber();

	// Do not call this function! Only BuggyRos calls this function!
	public void setSequenceNumber(long sequenceNumber);
	
}
