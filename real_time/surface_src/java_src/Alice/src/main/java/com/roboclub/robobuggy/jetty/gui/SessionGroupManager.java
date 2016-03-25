package com.roboclub.robobuggy.jetty.gui;

import java.util.concurrent.ConcurrentHashMap;

import org.eclipse.jetty.websocket.api.Session;

/**
 * Singleton class which manages the client / server group system.
 * @author Vasu Agrawal
 *
 */
public class SessionGroupManager {
	
	private ConcurrentHashMap<String, ConcurrentHashMap<Integer, Session>> clients;
	private int clientCount = 0;
	
	/**
	 * Simple constructor
	 */
	public SessionGroupManager() {
		clients = new ConcurrentHashMap<String, ConcurrentHashMap<Integer, Session>>();
	}
	
	/**
	 * Client accessor
	 * @return client object
	 */
	public ConcurrentHashMap<String, ConcurrentHashMap<Integer, Session>> getClients() {
		return clients;
	}
	
	/**
	 * 
	 * @param groupName name of the group to get
	 * @return Group of clients in the given group, or null if group doesn't exist
	 */
	public synchronized ConcurrentHashMap<Integer, Session> getGroup(String groupName) {
		// may want to instead return an empty group 
		return clients.get(groupName.toLowerCase());
	}
	
	/**
	 * Make a new group with groupName if it doesn't exist.
	 * @param groupName name of the group to create
	 */
	public void newGroup(String groupName) {
		clients.putIfAbsent(groupName.toLowerCase(), new ConcurrentHashMap<Integer, Session>());
	}
	
	/**
	 * Remove the session with ID from the manager.
	 * Nothing will ever have an id of 0.
	 * @param id id of session to remove
	 * @return session object or null
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
	 * @param session Which session to add
	 * @return client count
	 */
	public synchronized int addSessionToGroup(String groupName, Session session) {
		clientCount++;
		newGroup(groupName.toLowerCase());
		clients.get(groupName.toLowerCase()).put(clientCount, session);
		return clientCount;
	}
}