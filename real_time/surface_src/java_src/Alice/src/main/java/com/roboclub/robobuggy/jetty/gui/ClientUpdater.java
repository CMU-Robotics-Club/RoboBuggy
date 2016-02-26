package com.roboclub.robobuggy.jetty.gui;

import java.lang.reflect.Modifier;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import org.eclipse.jetty.websocket.api.Session;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * acts as a data hub for robobuggy messages, sends them off to the multiple clients
 */
public class ClientUpdater {
	
	// Shouldn't actually need to be threadsafe, I don't think it gets accessed anywhere else
	private static LinkedBlockingQueue<String> updates = new LinkedBlockingQueue<String>();
	private Gson messageTranslator;

	/**
	 * starts a new client updater
	 */
	public ClientUpdater() {
            messageTranslator = new GsonBuilder()
                                    .excludeFieldsWithModifiers(Modifier.TRANSIENT)
                                    .serializeSpecialFloatingPointValues()
                                    .create()
                                    ;

		
		// Once we're subscribed to all the things, we shouldn't need to do more other than push.
        for (NodeChannel filter : NodeChannel.getLoggingChannels()) {
            new Subscriber(filter.getMsgPath(), new MessageListener() {
                @Override
                public void actionPerformed(String topicName, Message m) {
                	
                    try {
                        String msgAsJsonString = messageTranslator.toJson(m);
                        updates.put(msgAsJsonString);
                    } catch (Exception e) {
                    	System.out.println("Exception in translation");
                    }
                }
            });
        }
		
		
		Thread updater = new Thread() {
			
			private ConcurrentHashMap<Integer, Session> clients = WSHandler.getClients();
			private LinkedBlockingQueue<String> updates = ClientUpdater.updates;		
			
			public void run() {
				System.out.println("Running?");
				while (true) {
					try {
						String update = updates.take();
						System.out.println(update);
						Session session;
						
						for (int key : clients.keySet()) {
							session = clients.get(key);
							session.getRemote().sendString(update);
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
