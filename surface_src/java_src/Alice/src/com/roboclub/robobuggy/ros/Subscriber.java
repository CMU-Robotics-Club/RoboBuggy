package com.roboclub.robobuggy.ros;

import java.util.ArrayDeque;
import java.util.Deque;

import com.roboclub.robobuggy.ros.internal.MessageServer;

public class Subscriber {

	private MessageServer ms;
	private MessageListener callback;

	private Deque<Message> local_inbox = new ArrayDeque<Message>();
	// private Lock local_inbox_lock = new ReentrantLock();

	private Thread worker;

	// Note that callbacks will run in a different thread
	public Subscriber(String topic, MessageListener messageListener) {
		ms = MessageServer.getMaster();
		this.callback = messageListener;
		// Start a thread to pull from mqueue
		ms.addListener(topic, this);

		worker = new Thread(new WorkerThread(), "Subscriber-Internal");
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

			// while (true) {
			// Retrieve the latest measurements
			synchronized (local_inbox) {
				while (local_inbox.peek() != null) {
					try {
						local_inbox.wait();
					} catch (InterruptedException ie) {
						System.out
								.println("much awoken for no reason, such wow");
					}

					Message m = local_inbox.poll();
					if (m != null) {
						callback.actionPerformed(m);
					} else {
						System.out
								.println("It looks like there is a race condition in Subscriber");
					}
				}
			}
			// }
		}
	}
}