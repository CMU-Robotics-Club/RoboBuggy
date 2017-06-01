package com.roboclub.robobuggy.ros;

import com.roboclub.robobuggy.ros.internal.MessageServer;

/**
 * @author Matt Sebek
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

public class Publisher {

	private String topic_name;
	private MessageServer mserver;
	private long sequenceNumber = 0;
	
	public Publisher(String topic) {
		topic_name = topic;
		mserver = MessageServer.getMaster();
	}

	public void publish(Message m) {
		if(m == null) {
			System.out.println("message was null...skip publishing.");
			return;
		}
		m.setSequenceNumber(sequenceNumber);
		sequenceNumber++;
		mserver.sendMessage(topic_name, m);
	}
	
	public void close() {
		mserver = null;
	}

}
