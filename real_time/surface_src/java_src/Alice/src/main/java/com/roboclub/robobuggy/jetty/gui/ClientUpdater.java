package com.roboclub.robobuggy.jetty.gui;

import java.util.concurrent.LinkedBlockingQueue;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * Base class for all client updaters. Each updater will connect to a group and publish messages
 * to that group as desired.
 * @author Vasu Agrawal
 *
 */
public abstract class ClientUpdater {
	
	// Shouldn't actually need to be threadsafe, I don't think it gets accessed anywhere else
	protected LinkedBlockingQueue<Message> dataUpdates = new LinkedBlockingQueue<Message>();
	protected LinkedBlockingQueue<Message> stateUpdates = new LinkedBlockingQueue<Message>();
	
	/**
	 * Method to subscribe to all channels.
	 */
	protected void subscribeToAllData() {
		// Once we're subscribed to all the things, we shouldn't need to do more other than push.
        for (NodeChannel filter : NodeChannel.getLoggingChannels()) {
            new Subscriber(filter.getMsgPath(), new MessageListener() {
                @Override
                public void actionPerformed(String topicName, Message m) {
                	
                    try {
                    	dataUpdates.put(m);
                    } catch (Exception e) {
						new RobobuggyLogicNotification("unable to add to queue", RobobuggyMessageLevel.EXCEPTION);
                    }
                }
            });
        }
	}	
	
	/**
	 * Subscribe to only state messages, storing the channel.
	 */
	protected void subscribeToAllState() {
		// Once we're subscribed to all the things, we shouldn't need to do more other than push.
        for (NodeChannel filter : NodeChannel.getLoggingChannels()) {            
            new Subscriber(filter.getStatePath(), new MessageListener() {
            	@Override
            	public void actionPerformed(String topicName, Message m) {
            		try {
            			stateUpdates.put(m);
            		} catch (Exception e) {
            			new RobobuggyLogicNotification("Unable to subscribe to status for " + filter.getStatePath(), RobobuggyMessageLevel.EXCEPTION);
            		}
            	}
            });
        }
	}
	
	/**
	 * Simple constructor, only subscribes to all channels.
	 */
	public ClientUpdater() {
	}
}
