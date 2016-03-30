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

	class WorkerThread implements Runnable {

		@Override
		public void run() {
			Message m;
			while(true) {
				synchronized (local_inbox) {
					while (true) { try {
							local_inbox.wait();
							break;
						} catch (InterruptedException ie) {
							System.out.println("much awoken for no reason, such wow"); 
							//TODO fix trevor's sense of humor so he appreciates this
						}
					}
					// If we were not woken spuriously, then there must be an item in the queue. 
					m = local_inbox.poll();
				}	
                assert(m != null);
                // N. B. Do not hold local_inbox lock over user callback.
                // 		 If the callback acquires a lock, we may acquire locks out of order,
                // 		 This can lead to us having A and needing B, and them having B and needing A. 
                callback.actionPerformed(topicName, m);
            }
		}
	}
}