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

// Interface inspired by Sweing's action listener system
public interface MessageListener {

	// To use this, cast the received message to the correct type. 
	public void actionPerformed(Message m);
	
}
