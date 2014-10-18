package com.roboclub.robobuggy.serial;

import java.util.EventListener;

/**
 * 
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class SerialEvent implements EventListener {
	private char[] buffer;
	private int len;
	
	public SerialEvent(char[] buffer_, int len_) {
		buffer = buffer_;
		len = len_;
	}
	
	public char[] getBuffer() {
		return this.buffer;
	}
	
	public int getLength() {
		return this.len;
	}
}