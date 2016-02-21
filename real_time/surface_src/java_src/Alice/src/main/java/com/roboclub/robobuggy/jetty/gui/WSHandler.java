package com.roboclub.robobuggy.jetty.gui;

import java.io.IOException;
import java.util.concurrent.ConcurrentHashMap;

import org.eclipse.jetty.websocket.api.Session;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketClose;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketConnect;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketError;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketMessage;
import org.eclipse.jetty.websocket.api.annotations.WebSocket;

@WebSocket
public class WSHandler {
	
	static public ConcurrentHashMap<Integer, Session> clients = new ConcurrentHashMap<Integer, Session>();
	static private int clientCount = 0;
	private int clientID = 0;
	
    @OnWebSocketClose
    public void onClose(int statusCode, String reason) {
    	clients.remove(clientID);
    	clientCount--;
        System.out.println("Close: statusCode=" + statusCode + ", reason=" + reason);
    }

    @OnWebSocketError
    public void onError(Throwable t) {
    	clients.remove(clientID);
    	clientCount--;
        System.out.println("Error: " + t.getMessage());
    }

    @OnWebSocketConnect
    public void onConnect(Session session) {
    	// Save the session to be pushed to later
    	clientID = clientCount;
    	clients.put(clientID, session);
    	
        System.out.println("Connect: " + session.getRemoteAddress().getAddress());
        
        try {
            session.getRemote().sendString("Hello Webbrowser " + Integer.toString(clientCount));
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        clientCount++;
    }

    @OnWebSocketMessage
    public void onMessage(String message) {
        System.out.println("Message: " + message);
    }
}