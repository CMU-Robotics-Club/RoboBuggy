package com.roboclub.robobuggy.jetty.gui;

import java.util.concurrent.LinkedBlockingQueue;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

public abstract class ClientUpdater {
	
	// Shouldn't actually need to be threadsafe, I don't think it gets accessed anywhere else
	protected LinkedBlockingQueue<Message> updates = new LinkedBlockingQueue<Message>();
	
	private void subscribeToAll() {
		// Once we're subscribed to all the things, we shouldn't need to do more other than push.
        for (NodeChannel filter : NodeChannel.getLoggingChannels()) {
            new Subscriber(filter.getMsgPath(), new MessageListener() {
                @Override
                public void actionPerformed(String topicName, Message m) {
                	
                    try {
                    	updates.put(m);
                    } catch (Exception e) {
                    	System.out.println("Unable to add to queue");
                    }
                }
            });
        }	
	}
	
	public ClientUpdater() {
		subscribeToAll();
	}
}
