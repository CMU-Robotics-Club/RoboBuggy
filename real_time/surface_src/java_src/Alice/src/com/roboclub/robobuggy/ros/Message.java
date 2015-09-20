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

	// toLogString returns a string representing the message.
	// - does not have a new-line afterwards
	// - if any work needs to be done (e.g. save an image to disk, then
	// toLogString needs to start that work.
	public String toLogString();

	// Given the output of toLogString, re-hydrate a message from it.
	public Message fromLogString(String str);
}
