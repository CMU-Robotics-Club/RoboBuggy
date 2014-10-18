package com.roboclub.robobuggy.ros;

/**
 * @author Matt Sebek
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public interface Message {
	// TODO force messages to have a time
	public String toLogString();

	public void fromLogString(String str);
}
