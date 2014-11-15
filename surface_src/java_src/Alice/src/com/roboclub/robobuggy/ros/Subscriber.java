package com.roboclub.robobuggy.ros;

import java.util.ArrayDeque;
import java.util.Deque;

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

public class Subscriber {

	private MessageServer ms;
	private MessageListener callback;
	private String topicName;

	private Deque<Message> local_inbox = new ArrayDeque<Message>();
	// private Lock local_inbox_lock = new ReentrantLock();

	private Thread worker;

	// Note that callbacks will run in a different thread
	public Subscriber(String topic, MessageListener messageListener) {
		this.ms = MessageServer.getMaster();
		this.callback = messageListener;
		this.topicName = topic;

		// Register this thread as a subscriber to "topic" on the master queue
		ms.addListener(topic, this);

		worker = new Thread(new WorkerThread(), "Subscriber-Internal=" + topic);
		worker.start();
	}

	public void putMessage(Message m) {
		synchronized (local_inbox) {
			local_inbox.push(m);
			local_inbox.notify();
		}
	}

	// note that if we ever have more than 1 thread, we will have to worry
	// about the callback more.
	class WorkerThread implements Runnable {

		@Override
		public void run() {
			Message m;
			synchronized (local_inbox) {
				while (true) {
					while (local_inbox.peek() == null) {
						try {
							local_inbox.wait();
						} catch (InterruptedException ie) {
							System.out
									.println("much awoken for no reason, such wow");
						}
					}

					m = local_inbox.poll();
					if (m != null) {
						callback.actionPerformed(topicName, m);
					}
				}
			}
		}
	}
}