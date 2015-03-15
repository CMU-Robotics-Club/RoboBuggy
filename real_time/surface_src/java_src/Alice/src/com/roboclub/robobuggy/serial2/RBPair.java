package com.roboclub.robobuggy.serial2;

public class RBPair {
	private int a;
	private RBSerialMessage b;

	RBPair(int num_bytes_read, RBSerialMessage new_message) {
		a = num_bytes_read;
		b = new_message;
	}
		
	public int getNumber() {
		return a;
	}
		
	public RBSerialMessage getMessage() {
		return b;
	}
}
