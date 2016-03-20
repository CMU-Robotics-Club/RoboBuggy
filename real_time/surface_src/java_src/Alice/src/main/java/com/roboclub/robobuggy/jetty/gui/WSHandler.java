package com.roboclub.robobuggy.jetty.gui;

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
	private int clientID = 0;
	
	/**
	 * Returns Session Group Manager
	 * @return the sgm
	 */
	public static SessionGroupManager getSGM() {
		return sgm;
	}
	
	/**
	 * What to do if the websocket closes
	 * @param statusCode close code
	 * @param reason reason for closing
	 */
    @OnWebSocketClose
    public void onClose(int statusCode, String reason) {
    	sgm.removeSession(clientID);
//        System.out.println("Close: statusCode=" + statusCode + ", reason=" + reason + " ID: " + clientID);
    }

    /**
     * What to do on an error
     * @param t the error
     */
    @OnWebSocketError
    public void onError(Throwable t) {
    	sgm.removeSession(clientID);
//        System.out.println("Error: " + t.getMessage());
    }

    /**
     * what to do on connect
     * @param session client session
     */
    @OnWebSocketConnect
    public void onConnect(Session session) {
    	clientID = sgm.addSessionToGroup("unsorted", session);    	
//        System.out.println("Connect: " + session.getRemoteAddress().getAddress() + " ID: " + clientID);
    }

    /**
     * What to do on message
     * This allows a connected client to change the stream it's connected to
     * @param message the message we received
     */
    @OnWebSocketMessage
    public void onMessage(String message) {
    	Session session = sgm.removeSession(clientID);
    	
    	for (String groupName : sgm.getClients().keySet()) {
    		if (message.toLowerCase().contains(groupName)) {
    			clientID = sgm.addSessionToGroup(groupName,  session);
    			return;
    		}
    	}
    	
    	clientID = sgm.addSessionToGroup("unsorted", session);
    }
}