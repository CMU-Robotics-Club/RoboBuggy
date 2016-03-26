package com.roboclub.robobuggy.jetty.gui;

import java.lang.reflect.Modifier;
import java.util.concurrent.ConcurrentHashMap;

import org.eclipse.jetty.websocket.api.Session;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;

/**
 * Sends "data" updates to all connected clients.
 * In practice this will send everything except camera images, but it can be made more granular as necessary.
 * @author Vasu Agrawal
 *
 */
public class ClientDataUpdater extends ClientUpdater {
	
	private Gson messageTranslator;
	private Thread updater;
	
	/**
	 * Connect to the "data" subscriber group.
	 */
	public ClientDataUpdater() {
		super(); // subscribe to all the things
		WSHandler.getSGM().newGroup("data"); // listen for data
		
		try {
            messageTranslator = new GsonBuilder()
                                    .excludeFieldsWithModifiers(Modifier.TRANSIENT)
                                    .serializeSpecialFloatingPointValues()
                                    .create()
                                    ;
        } catch (Exception e) {
        	e.printStackTrace();
        }

		updater = new Thread() {
			
			private ConcurrentHashMap<Integer, Session> clients;	
			private Message update;
			private Session session;
			
			public void run() {
				while (true) {
					try {
						clients = WSHandler.getSGM().getGroup("data");
						update = updates.take();

						if (clients != null) {
							for (int key: clients.keySet()) {
								session = clients.get(key);
								session.getRemote().sendString(messageTranslator.toJson(update));
							}
						} else {
							new RobobuggyLogicNotification("NULL clients", RobobuggyMessageLevel.EXCEPTION);
						}
					
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			}
		};
		
		updater.start();
	}
}