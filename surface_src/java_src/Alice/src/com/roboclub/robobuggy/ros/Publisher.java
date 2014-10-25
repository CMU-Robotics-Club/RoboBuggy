package com.roboclub.robobuggy.ros;

import com.roboclub.robobuggy.ros.internal.MessageServer;

/**
 * @author Matt Sebek
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class Publisher {

	private String topic_name;
	private MessageServer mserver;
	
	public Publisher(String topic) {
		topic_name = topic;
		mserver = MessageServer.getMaster();
	}
	
	public void publish(Message m) {
		mserver.sendMessage(topic_name, m);
	}
	
}
