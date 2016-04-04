package com.roboclub.robobuggy.jetty.gui;

import java.util.concurrent.ConcurrentHashMap;

import org.eclipse.jetty.websocket.api.Session;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketClose;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketConnect;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketError;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketMessage;
import org.eclipse.jetty.websocket.api.annotations.WebSocket;

/**
 * Web socket handler class.
 * @author Vasu Agrawal
 *
 */
@WebSocket
public class WSHandler {
	
	private static SessionGroupManager sgm = new SessionGroupManager();
	private static ConcurrentHashMap<String, CommandCallback> cm = new ConcurrentHashMap<String, CommandCallback>();
	private int clientID = 0;
	
	private static final String GROUP_STRING = "group:";
	private static final String COMMAND_STRING = "command:";
	
	/**
	 * Returns Session Group Manager
	 * @return the sgm
	 */
	public static SessionGroupManager getSGM() {
		return sgm;
	}
	
	/**
	 * Get the command grouping
	 * @return cm
	 */
	public static ConcurrentHashMap<String, CommandCallback> getCM() {
		return cm;
	}
	
	/**
	 * What to do if the websocket closes
	 * @param statusCode close code
	 * @param reason reason for closing
	 */
    @OnWebSocketClose
    public void onClose(int statusCode, String reason) {
    	sgm.removeSession(clientID);
    }

    /**
     * What to do on an error
     * @param t the error
     */
    @OnWebSocketError
    public void onError(Throwable t) {
    	sgm.removeSession(clientID);
    }

    /**
     * what to do on connect
     * @param session client session
     */
    @OnWebSocketConnect
    public void onConnect(Session session) {
    	clientID = sgm.addSessionToGroup("unsorted", session);    	
    }

    /**
     * What to do on message
     * This allows a connected client to change the stream it's connected to
     * @param message the message we received
     */
    @OnWebSocketMessage
    public void onMessage(String message) {
    	if (message.toLowerCase().contains(GROUP_STRING)) {
    		// resort the client if necessary / possible
	    	Session session = sgm.removeSession(clientID);
	    	for (String groupName : sgm.getClients().keySet()) {
	    		if (message.toLowerCase().contains(groupName)) {
	    			clientID = sgm.addSessionToGroup(groupName, session);
	    			return;
	    		}
	    	}
	    	
	    	clientID = sgm.addSessionToGroup("unsorted", session);
    	} else if (message.toLowerCase().contains(COMMAND_STRING)) {
    		// handle commands with whatever command handlers we have so far
    		for (String command : cm.keySet()) {
    			if (message.contains(command)) {
    				cm.get(command).callback(message, clientID);
    				return;
    			}
    		}
    		// treat the message as a command
    	}
    }
}