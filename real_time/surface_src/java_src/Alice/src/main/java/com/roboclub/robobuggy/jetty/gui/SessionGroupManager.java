package com.roboclub.robobuggy.jetty.gui;

import java.util.concurrent.ConcurrentHashMap;

import org.eclipse.jetty.websocket.api.Session;

public class SessionGroupManager {
	
	private ConcurrentHashMap<String, ConcurrentHashMap<Integer, Session>> clients;
	private int clientCount = 0;
	

	public SessionGroupManager() {
		clients = new ConcurrentHashMap<String, ConcurrentHashMap<Integer, Session>>();
	}
	
	public ConcurrentHashMap<String, ConcurrentHashMap<Integer, Session>> getClients() {
		return clients;
	}
	
	/**
	 * 
	 * @param groupName
	 * @return Group of clients in the given group, or null if group doesn't exist
	 */
	public ConcurrentHashMap<Integer, Session> getGroup(String groupName) {
		// may want to instead return an empty group 
		return clients.get(groupName.toLowerCase());
	}
	
	/**
	 * Make a new group with groupName doesn't exist.
	 * @param groupName
	 */
	public void newGroup(String groupName) {
		if (!clients.containsKey(groupName.toLowerCase())) {
			clients.put(groupName.toLowerCase(), new ConcurrentHashMap<Integer, Session>());
		}
	}
	
	/**
	 * Remove the session with ID from the manager.
	 * Nothing will ever have an id of 0.
	 * @param id
	 */
	public synchronized Session removeSession(int id) {
		Session session;
		clientCount--;
		
		for (ConcurrentHashMap<Integer, Session> map : clients.values()) {
			session = map.remove(id);
			if (session != null) {
				return session;
			}
		}
		return null;
	}
	
	/**
	 * Adds session to groupName. If groupName doesn't exist, a new group will be created.
	 * @param groupName Case insensitive group name.
	 * @param session
	 * @return
	 */
	public synchronized int addSessionToGroup(String groupName, Session session) {
		clientCount++;
		newGroup(groupName.toLowerCase());
		clients.get(groupName.toLowerCase()).put(clientCount, session);
		return clientCount;
	}
}