package com.roboclub.robobuggy.ros;

import java.util.ArrayDeque;
import java.util.Deque;

import com.roboclub.robobuggy.ros.internal.MessageServer;

/**
 * @author Matt Sebek
 *
 * @version 0.5
 *
 *          CHANGELOG: NONE
 *
 *          DESCRIPTION: Note that we add elements to the head of the queue, and
 *          take elements from the tail of the queue.
 */

public class Subscriber {

	private MessageServer ms;
	private MessageListener callback;
	private String topicName;
	private int maxLocalInboxLength;
	private long messagesReceived = 0;
	private long messagesDropped = 0;
	
	private Deque<Message> local_inbox = new ArrayDeque<Message>();

	private volatile boolean shouldShutdown = false;
	private Thread worker;

	// Note that callbacks will run in a different thread
	public Subscriber(String ownerName, String topic, int maxMessagesToKeep, MessageListener messageListener) {
		this.ms = MessageServer.getMaster();
		this.callback = messageListener;
		this.topicName = topic;
		this.maxLocalInboxLength = maxMessagesToKeep;

		// Register this thread as a subscriber to "topic" on the master queue
		ms.addListener(topic, this);

		worker = new Thread(new WorkerThread(), "Subscriber=" + ownerName + ":" + topic);
		worker.start();
	}

	// Note that callbacks will run in a different thread
	public Subscriber(String ownerName, String topic, MessageListener messageListener) {
		this(ownerName, topic, Integer.MAX_VALUE, messageListener);
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
			while (true) {
				// Grab the lock and check 2 cases:
				// 1. if there is a message in the queue, process this message
				// immediately.
				// 2. if not, drop the lock and wait for someone to notify us
				// that work has been added to the queue.
				// if we are woken spuriously, go back to sleep.
				synchronized (local_inbox) {
					while (local_inbox.size() == 0) {
						try {
							local_inbox.wait();
						} catch (InterruptedException ie) {
							if(!shouldShutdown) {
								System.out.println("Interrupted but not shutting down? Impossible.");
								continue;
							}
							break;
						}
					}
					if(shouldShutdown) {
						return;
					}
					// Note that we compare with size()-1; if maxLocalInboxLength is 1, we
					// want to leave one message in the inbox to poll outside of the loop.
					while((local_inbox.size()-1) > maxLocalInboxLength) {
						// Remove items from local_inbox until the length is equal to max.
						local_inbox.pollLast();
						messagesDropped++;
					}
					m = local_inbox.pollLast();
				}
				if (m == null) {
					System.out.println("If length is zero, we cannot be null... Very bad.");
				}
				// N. B. Do not hold local_inbox lock over user callback.
				// If the callback acquires a lock, we may acquire locks out of
				// order,
				// This can lead to us having A and needing B, and them having B
				// and needing A.
				messagesReceived++;
				callback.actionPerformed(topicName, m);
			}
		}
	}
	
	// Stop receiving callbacks from Master. 
	// Used for cleaning up this node after a test completes; subscribers will not be cleaned up
	// automatically because MessageServer holds a reference to the Subscriber.
	public void close() {
		ms.removeListener(this.topicName, this);
		this.ms = null;
		this.shouldShutdown = true;
		worker.interrupt();
		try {
			worker.join();
		} catch (InterruptedException e) {
			
			e.printStackTrace();
		}
	}

	// Note that these methods allow the subscriber callback to call back 
	//  into the class while the callback is running. Since no locks are held 
	//  over the callback, it is safe to acquire a lock here.
	public int getMessageQueueLength() {
		synchronized (local_inbox) {
			return local_inbox.size();
		}
	}

	// TODO: do we need to hold this lock if we instead make this variable
	// volatile? Will it work on the same core? on different cores? 
	public long getNumberMessagesDropped() {
		synchronized (local_inbox) {
			return messagesDropped;
		}
	}

}