package com.roboclub.robobuggy.jetty.gui;

/**
 * Blank class to be used as function callbacks from web gui.
 */
public abstract class CommandCallback {

	/**
	 * Method to override with actual callback.
	 * @param message the message from the client
	 * @param clientID the id of the client, get from SGM.
	 */
	public abstract void callback(String message, int clientID);
}
