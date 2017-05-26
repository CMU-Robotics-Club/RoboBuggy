package com.roboclub.robobuggy;

import java.util.concurrent.Semaphore;

// A simple circular buffer. 
// This is (will be) thread-safe if you are careful. 
// Note that 
public class CircularBuffer {

	
	private static byte[] buffer;
	private final Semaphore mutex = new Semaphore(1); 
	private int head; // index of the first occupied element in the queue. Enqueue here
	private int tail; // index of the last occupied element in the queue. Dequeue here
	public CircularBuffer(int length) {
		buffer = new byte[length];
		head = 0;
		tail = 0;
	}
	// TODO: add multiple items to the queue at once to prevent mutex contention
	public void enqueue(byte b) {
		mutex.acquireUninterruptibly();
		
		if((head+1)%buffer.length == tail) {
			// Queue is full
			mutex.release();
			throw new RuntimeException("Full");
		}
		
		buffer[head] = b;
		head = (head+1) % buffer.length;
		mutex.release();
	}
	
	
	
	// throws not found exception
	public byte dequeue() {
		mutex.acquireUninterruptibly();
		if(head==tail) {
			// Queue is empty
			mutex.release();
			throw new RuntimeException("No things in queue");
			// does not get past here
		} 
		
		byte b = buffer[tail];
		tail = (tail+1) % buffer.length;
		
		mutex.release();
		return b;
	}
	
}
