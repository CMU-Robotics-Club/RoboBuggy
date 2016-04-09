package com.roboclub.robobuggy.ros.internal;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * @author Matt Sebek
 * @author Trevor Decker
 *
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: This class is the singleton that backs all publishers/subscribers.
 *    MessageServer is a global message queue that all publishers publish to. 
 *    The MailmanThread pulls messages from this global message queue, and puts
 *    the individual messages into the queues that each Subscriber maintains.
 */

// This class is the backbone of the publisher/subscriber infrastructure.
public class MessageServer {

	private static MessageServer _master;
	private static Lock singleton_lock = new ReentrantLock();

	private Map<String, List<Subscriber>> outbox_mapping = new HashMap<String, List<Subscriber>>();
	private ReadWriteLock outbox_lock = new ReentrantReadWriteLock(true);

	// Contains all messages that threads want sent.
	private LinkedBlockingQueue<Map.Entry<String, Message>> inbox = new LinkedBlockingQueue<Map.Entry<String, Message>>();

	private int messagesProcessed = 0;
	
	// Picks up post, and delivers it.
	private class MailmanThread implements Runnable {

		@Override
		public void run() {
			while (true) {
				Map.Entry<String, Message> request;

				try {
					request = inbox.take();
				} catch (InterruptedException ie) {
					System.out.println("I don't think you can get here...");
					continue;
				}
				String topicName = request.getKey();
				Message m = request.getValue();
                		m.setTopicName(topicName);

				outbox_lock.readLock().lock();
				List<Subscriber> subs = outbox_mapping.get(topicName);
				// TODO: if the string ends in '/', then add logic that walks
				// through the subscribers and hits all the relevant ones.

				if (subs != null) {
					for (Subscriber s : subs) {
						// Note that this will block on s's inbox lock.
						s.putMessage(m);
					}
				}
				outbox_lock.readLock().unlock();
			}
		}

	}

	// Puts message in mailbox. Will be handled later.
	// Note that this will block on inbox's lock.
	public synchronized void sendMessage(String s, Message m) {
		AbstractMap.SimpleEntry<String, Message> am = new AbstractMap.SimpleEntry<String, Message>(s, m);
		if(!inbox.offer(am)) {
			System.out.println("Offer failed; dropping message?");
		}
	}


	
	/*
	 * The StatisticsThread 
	 */
	private class StatisticsThread implements Runnable {
		// Interesting things: how many messages are in the queue, 
		//					   how many messages processed since last time.
		
		public StatisticsThread(int pollingPeriodInMs) {
			
		}
		
		@Override
		public void run() {
		
		
		}
		
		
	}
	
	private MessageServer() {
		(new Thread(new MailmanThread(), "MessageServer-loop")).start();
	}

	public static MessageServer getMaster() {
		singleton_lock.lock();
		if (_master == null) {
			_master = new MessageServer();
		}
		singleton_lock.unlock();
		return _master;
	}

	// Will be called from multiple threads
	public void addListener(String s, Subscriber ml) {
		// Check if topic exists
		outbox_lock.writeLock().lock();
		List<Subscriber> listeners = outbox_mapping.get(s);
		if (listeners == null) {
			List<Subscriber> newTopic = new ArrayList<Subscriber>();
			newTopic.add(ml);
			outbox_mapping.put(s, newTopic);
		} else {
			listeners.add(ml);
		}
		outbox_lock.writeLock().unlock();
	}
	
	public void removeListener(String s, Subscriber ml) {
		outbox_lock.writeLock().lock();
		List<Subscriber> listeners = outbox_mapping.get(s);
		listeners.remove(ml);
		if(listeners.size() == 0) {
			outbox_mapping.remove(s, listeners);
		}
		outbox_lock.writeLock().unlock();
		
	}
	
}
