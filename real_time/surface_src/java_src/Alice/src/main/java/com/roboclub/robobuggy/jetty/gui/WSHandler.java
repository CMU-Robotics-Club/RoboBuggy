package com.roboclub.robobuggy.jetty.gui;

import java.io.IOException;
import java.util.concurrent.ConcurrentHashMap;

import org.eclipse.jetty.websocket.api.Session;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketClose;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketConnect;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketError;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketMessage;
import org.eclipse.jetty.websocket.api.annotations.WebSocket;
import org.eclipse.jetty.websocket.jsr356.decoders.IntegerDecoder;

/**
 * websocket handler
 */
@WebSocket
public class WSHandler {
	
	private static ConcurrentHashMap<Integer, Session> clients = new ConcurrentHashMap<Integer, Session>();
	private int clientCount = 0;
	private int clientID = 0;

    /**
     * @return the clients in the current session
     */
    public static ConcurrentHashMap<Integer, Session> getClients() {
        return clients;
    }

    /**
     * we needed to close connection to this client
     * @param statusCode client status
     * @param reason client reason
     */
    @OnWebSocketClose
    public void onClose(int statusCode, String reason) {
    	clients.remove(clientID);
    	clientCount--;
        System.out.println("Close: statusCode=" + statusCode + ", reason=" + reason);
    }

    /**
     * client had an error
     * @param t the error that occurred
     */
    @OnWebSocketError
    public void onError(Throwable t) {
    	clients.remove(clientID);
    	clientCount--;
        System.out.println("Error: " + t.getMessage());
    }

    /**
     * new client connected
     * @param session session that connected
     */
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

    /**
     * new message happened
     * @param message new message
     */
    @OnWebSocketMessage
    public void onMessage(String message) {
        System.out.println("Message: " + message);
    }
}