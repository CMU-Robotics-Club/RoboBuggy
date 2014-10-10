package com.roboclub.robobuggy.ros;

import com.roboclub.robobuggy.ros.internal.MessageServer;

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
