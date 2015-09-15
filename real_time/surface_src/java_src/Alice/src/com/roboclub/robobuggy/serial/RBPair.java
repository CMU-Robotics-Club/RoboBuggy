package com.roboclub.robobuggy.serial;

public class RBPair {
	private int num_bytes_read;
	private RBSerialMessage rb_message;

	RBPair(int num_bytes_read, RBSerialMessage new_message) {
		this.num_bytes_read = num_bytes_read;
		this.rb_message = new_message;
	}
		
	public int getNumberOfBytesRead() {
		return num_bytes_read;
	}
		
	public RBSerialMessage getMessage() {
		return rb_message;
	}

	public int get_num_bytes_read() {
		// TODO Auto-generated method stub
		return 0;
	}
}
