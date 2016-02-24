package com.roboclub.robobuggy.jetty.gui;

import java.lang.reflect.Modifier;
import java.util.concurrent.ConcurrentHashMap;
import org.eclipse.jetty.websocket.api.Session;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.roboclub.robobuggy.ros.Message;

public class ClientDataUpdater extends ClientUpdater {
	
	private Gson messageTranslator;
	private Thread updater;
	
	public ClientDataUpdater() {
		super(); // subscribe to all the things
		WSHandler.sgm.newGroup("data"); // listen for data
		
		try {
            messageTranslator = new GsonBuilder()
                                    .excludeFieldsWithModifiers(Modifier.TRANSIENT)
                                    .serializeSpecialFloatingPointValues()
                                    .create()
                                    ;
        } catch (Exception e) {
        }

		updater = new Thread() {
			
			ConcurrentHashMap<Integer, Session> clients;	
			private Message update;
			private Session session;
			
			public void run() {
				while (true) {
					try {
						clients = WSHandler.sgm.getGroup("data");
						update = updates.take();

						if (clients != null) {
							for (int key: clients.keySet()) {
								session = clients.get(key);
								session.getRemote().sendString(messageTranslator.toJson(update));
							}
						} else {
							System.out.println("Null clients");
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