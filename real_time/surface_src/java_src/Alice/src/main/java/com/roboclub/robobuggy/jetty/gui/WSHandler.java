package com.roboclub.robobuggy.jetty.gui;

import org.eclipse.jetty.websocket.api.Session;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketClose;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketConnect;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketError;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketMessage;
import org.eclipse.jetty.websocket.api.annotations.WebSocket;

@WebSocket
public class WSHandler {
	
	static public SessionGroupManager sgm = new SessionGroupManager();
	private int clientID = 0;
		
    @OnWebSocketClose
    public void onClose(int statusCode, String reason) {
    	sgm.removeSession(clientID);
        System.out.println("Close: statusCode=" + statusCode + ", reason=" + reason);
    }

    @OnWebSocketError
    public void onError(Throwable t) {
    	sgm.removeSession(clientID);
        System.out.println("Error: " + t.getMessage());
    }

    @OnWebSocketConnect
    public void onConnect(Session session) {
    	clientID = sgm.addSessionToGroup("unsorted", session);    	
        System.out.println("Connect: " + session.getRemoteAddress().getAddress() + " ID: " + clientID);
    }

    // Switch own socket grouping here
    @OnWebSocketMessage
    public void onMessage(String message) {
    	System.out.println(message);
    	Session session = sgm.removeSession(clientID);
    	
    	for (String groupName : sgm.getClients().keySet()) {
    		if (message.toLowerCase().contains(groupName)) {
    			sgm.addSessionToGroup(groupName,  session);
    			return;
    		}
    	}
    	
    	sgm.addSessionToGroup("unsorted", session);
    }
}