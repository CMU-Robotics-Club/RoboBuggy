package com.roboclub.robobuggy.serial2;

import gnu.io.SerialPort;

import java.io.InputStream;
import java.io.OutputStream;

import com.roboclub.robobuggy.ros.Node;

// Properly initializes the serial things
public abstract class SerialNode implements Node {
	
	SerialPort sp;
	InputStream serial_input;
	OutputStream serial_output;
	
	String thread_name;
	Thread io_thread;
	
	boolean running = false;


	// IO does not start until a serial port is attached
	public SerialNode(String thread_name) {
		this.thread_name = thread_name;
	}

	
	public void setSerialPort(SerialPort sp) {
		this.sp = sp;
		// Set port to be the right baud rate
		// Set port to be non-blocking
		// Set port to have a reasonable input buffer size
		
		// Begin the madness
		start_io();
	}

	@Override
	public boolean shutdown() {
		running = false;
		// Node will commence shutdown next loop.
	
		// Wait forever to join...hope that we're not seized up...
		try {
			io_thread.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
			return false;
		}
		return true;
	}

	// Once this is called, the real IO starts
	private void start_io() {
		io_thread = new Thread(new iothread(), thread_name + "io");
		io_thread.setPriority(Thread.MAX_PRIORITY);
		io_thread.start();
	}
	
	
	// Return true iff the data type is readable by this current node
	public abstract boolean matchDataSample(byte[] sample);

	// Return the number of bytes needed to match 
	public abstract int matchDataMinSize();


	// Return the expected baud rate of the current device
	public abstract int baudRate();
	
	public abstract void peel(byte[] buffer, int start, int bytes_available);

	private class iothread implements Runnable {
		@Override
		public void run() {
			// Read some from the port.
			read()
		}
	}
	
}
