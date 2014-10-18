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
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

// This class is the backbone of the publisher/subscriber infrastructure.
public class MessageServer{

	private static MessageServer _master;
	private static Lock singleton_lock = new ReentrantLock();

	
	private Map<String, List<Subscriber>> outbox_mapping = new HashMap<String, List<Subscriber>>();
	private ReadWriteLock outbox_lock = new ReentrantReadWriteLock(true); 

	
	// Contains all messages that threads want sent.
	// TODO: confirm threadsafety. 
	private LinkedBlockingQueue<Map.Entry<String, Message>> inbox = new LinkedBlockingQueue<Map.Entry<String, Message>>();

	// Picks up post, and delivers it.
	private class MailmanThread implements Runnable {

		@Override
		public void run() {
			while(true) {

				Map.Entry<String, Message> request;
				
				try { 
					request = inbox.take(); 
				} catch (InterruptedException ie) {
					System.out.println("I don't think you can get here...");
					continue;
				}
			
				String topicName = request.getKey();
				Message m = request.getValue();
			
				outbox_lock.readLock().lock();
				List<Subscriber> subs = outbox_mapping.get(topicName);
				
				if(subs != null) {
					for(Subscriber s : subs) {
						s.putMessage(m);
					}
				}
				outbox_lock.readLock().unlock();
			}	
		}
		
		
	}
	
	private MessageServer() {
		(new Thread(new MailmanThread(), "MessageServer-loop")).start();
	}
	
	public static MessageServer getMaster() {
		// TODO: this is broken for multithreading. consider the right way to do this. 
		singleton_lock.lock();
		if(_master == null) {
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
		if(listeners == null) {
			List<Subscriber> newTopic = new ArrayList<Subscriber>();
			newTopic.add(ml);
			outbox_mapping.put(s, newTopic);
		} else {
			listeners.add(ml);
		}
		outbox_lock.writeLock().unlock();
	}
	
	// Puts message in mailbox. Will be handled later.
	public void sendMessage(String s, Message m) {
		inbox.add(new AbstractMap.SimpleEntry<String, Message>(s, m));		
	}
}