package com.roboclub.robobuggy.serial2;

import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import com.roboclub.robobuggy.ros.Node;

// Properly initializes the serial things
public abstract class SerialNode implements Node {
	
	// Initialize first
	String thread_name;

	// Initialize after getting a serial port
	SerialPort sp;
	InputStream serial_input;
	OutputStream serial_output;
	
	Thread io_thread;
	
	boolean running = false;
	byte[] buf = new byte[128];
	int start = 0;
	int num_bytes = 0;

	// IO does not start until a serial port is attached
	public SerialNode(String thread_name) {
		this.thread_name = thread_name;
	}


	public boolean send(byte[] bytes) {
		if(serial_output == null) {
			return false;
		}
	
		try {
			serial_output.write(bytes);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return true;
		
	}
	
	public void setSerialPort(SerialPort sp) {
		this.sp = sp;
		// Set port to be the right baud rate
		int baudRate = baudRate();
		try {
			sp.setSerialPortParams(baudRate,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
		} catch (UnsupportedCommOperationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return;
		}
		
		
		// Set port to be non-blocking....maybe.
		sp.disableReceiveTimeout();
		sp.disableReceiveThreshold();
		// Set port to have a reasonable input buffer size?
	
		try {
			serial_input = sp.getInputStream();
			serial_output = sp.getOutputStream();
		} catch (IOException e) {
			System.out.println("broken");
		}
		
		// Begin the madness
		io_thread = new Thread(new iothread(), thread_name + "-serial");
		io_thread.setPriority(Thread.MAX_PRIORITY);
		io_thread.start();
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

	// Return true iff the data type is readable by this current node
	public abstract boolean matchDataSample(byte[] sample);

	// Return the number of bytes needed to match 
	public abstract int matchDataMinSize();


	// Return the expected baud rate of the current device
	public abstract int baudRate();

	// Peel is called once. user should read as many messages as possible
	// returns the number of bytes read; or 1 on failure
	public abstract int peel(byte[] buffer, int start, int bytes_available);
	
	private class iothread implements Runnable {
		int asleep_time = (1000 / 60);
		
		@Override
		public void run() {
			while(true) {
				try {
					num_bytes += serial_input.read(buf, start + num_bytes, buf.length - num_bytes); 
					//System.out.printf(new String(buf));
					//System.out.printf("%d\n", bytes);
				} catch (IOException e) {
					// TODO handle this error reasonably.
					e.printStackTrace();
				}
				int messages_read = 0;
				
				// Try to send things?
				
				
				while(true) {
					int num_read = peel(buf, start, num_bytes);
					if(num_read == 0) break;
					start += num_read;
					num_bytes -= num_read;
					messages_read++;
				}
				/*if(messages_read == 0) {
					System.out.print('0');
				}
				if(messages_read == 2) {
					System.out.print('2');
				}
				if(messages_read == 3) {
					System.out.print('3');
				}*/
				// Shift the array by the amount that we read.
				// TODO this is stupid and should be fixed
				for(int i = 0; i < num_bytes; i++) {
					buf[i] = buf[start+i];
				}
				start = 0;
			
			
				try {
					Thread.sleep(asleep_time);
				} catch (InterruptedException e) {
					System.out.println("sleep...interrupted?");
				}
			}
		}			
		}
	}