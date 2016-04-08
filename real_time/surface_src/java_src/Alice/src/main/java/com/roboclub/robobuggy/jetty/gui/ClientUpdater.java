package com.roboclub.robobuggy.jetty.gui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

import java.util.concurrent.LinkedBlockingQueue;

/**
 * Base class for all client updaters. Each updater will connect to a group and publish messages
 * to that group as desired.
 * @author Vasu Agrawal
 *
 */
public abstract class ClientUpdater {
	
	// Shouldn't actually need to be threadsafe, I don't think it gets accessed anywhere else
	protected LinkedBlockingQueue<Message> updates = new LinkedBlockingQueue<Message>();
	
	/**
	 * Method to subscribe to all channels.
	 */
	private void subscribeToAll() {
		// Once we're subscribed to all the things, we shouldn't need to do more other than push.
        for (NodeChannel filter : NodeChannel.getLoggingChannels()) {
            new Subscriber("jetty", filter.getMsgPath(), new MessageListener() {
                @Override
                public void actionPerformed(String topicName, Message m) {
                	
                    try {
                    	updates.put(m);
                    } catch (Exception e) {
						new RobobuggyLogicNotification("unable to add to queue", RobobuggyMessageLevel.EXCEPTION);
                    }
                }
            });
        }	
	}
	
	/**
	 * Simple constructor, only subscribes to all channels.
	 */
	public ClientUpdater() {
		subscribeToAll();
	}
}
