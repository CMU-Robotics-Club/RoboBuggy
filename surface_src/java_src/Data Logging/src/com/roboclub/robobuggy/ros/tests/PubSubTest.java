package com.roboclub.robobuggy.ros.tests;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

public class PubSubTest {

	public static void main(String[] args) {
		new PubSubTest();
	}

	public PubSubTest() {
		/*MessageListener ml = new MessageListener() {
			@Override
			public void actionPerformed(Message m) {
				return;
			}
		}*/
		(new Thread(new pubRunnable())).start();
		
		Subscriber s = new Subscriber("/hello", new MessageListener() {
			@Override 
			public void actionPerformed(Message m) {
				System.out.println(m);
			}
		});
		
	}
	
	public class pubRunnable implements Runnable {
		
		@Override
		public void run() {
			Publisher p = new Publisher("/hello");
			
			// Normally, this would be an ImuMeasurement,GpsMeasurement, etc. 
			Message m = new Message() { public String h = "hello!"; };
			
			while(true) {
				p.publish(m);
				try { Thread.sleep(1000);} catch (InterruptedException e) {
					e.printStackTrace();
				}
				// TODO also try this with wait(1000). Race-conditions difference?
			}
			
		}
		
		
		
	}
}
