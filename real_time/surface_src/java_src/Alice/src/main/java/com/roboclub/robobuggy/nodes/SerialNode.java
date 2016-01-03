package com.roboclub.robobuggy.nodes;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.ros.Node;

// Properly initializes the serial things
public abstract class SerialNode extends BuggyDecoratorNode {
	
	private final static int TIMEOUT = 2000;
	//Need to manage real, simulated, and calculated sensors
	
	// Initialize first
	String thread_name;
	public SerialPort sp;

	// Initialize after getting a serial port
	InputStream serial_input;
	OutputStream serial_output;
	
	Thread io_thread;
	
	boolean running = false;
	byte[] buf = new byte[128];
	int start = 0;
	int num_bytes = 0;

	/**
	 * Creates a {@link SerialNode} decorator for the specified {@link Node}
	 * @param base {@link Node} to decorate
	 * @param thread_name name of the thread
	 * @param portName name of the desired serial port
	 * @param baudRate baud rate of the serial port
	 */
	public SerialNode(Node base, String thread_name, String portName,
			int baudRate) {
		super(base);
		this.thread_name = thread_name;
		this.sp = connect(portName, baudRate);
	}
	
	// Open a serial port
	// Returns null if unable to connect, otherwise SerialPort
	private static SerialPort connect(String portName, int baudRate) {
		try {
	        CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
	        
	        if ( portIdentifier.isCurrentlyOwned() ) {
	        	System.err.println("Error: Port currently in use");
	        } else { 
	            CommPort commPort = portIdentifier.open(portName, TIMEOUT);
	            
	            if ( commPort instanceof SerialPort ) {
	                SerialPort serialPort = (SerialPort) commPort;
	                serialPort.setSerialPortParams(baudRate,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
	                return serialPort;
	            }
	        }
		} catch (Exception e) {
			System.err.println("Error: Unable to connect to port" + portName);
		}
		
		return null;
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
	
	/**
	 * Sets up a new thread to read in the serial data
	 * {@inheritDoc}*/
	@Override
	public boolean startDecoratorNode() {

		// Set port to be the right baud rate
		int baudRate = baudRate();
		try {
			sp.setSerialPortParams(baudRate,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
		} catch (UnsupportedCommOperationException e) {
			// TODO Auto-generated catch block
			System.out.println("Unsupported communication operation over serial port.");
			e.printStackTrace();
			return false;
		} catch (NullPointerException e) {
			System.out.println("Null Pointer Exception, unable to initialize the serial port. "
					+ "Printing stack trace below: \n");
			e.printStackTrace();
			return false;
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
			return false;
		}
		
		//Set the node to running
		running = true;
		
		// Begin the madness
		io_thread = new Thread(new iothread(), thread_name + "-serial");
		io_thread.setPriority(Thread.MAX_PRIORITY);
		io_thread.start();
		
		return true;
	}

	/**
	 * Shuts down the Serial reader thread
	 * {@inheritDoc}*/
	@Override
	public boolean shutdownDecoratorNode() {
		running = false;
		// iothread will commence shutdown next loop.
	
		// Wait forever to join...hope that we're not seized up...
		if (io_thread != null) {
			try {
				io_thread.join();
			} catch (InterruptedException e) {
				e.printStackTrace();
				return false;
			}
		}
		return true;
	}

	// Return true iff the data type is readable by this current node
	public abstract boolean matchDataSample(byte[] sample);

	// Return the number of bytes needed to match 
	public abstract int matchDataMinSize();


	// Return the expected baud rate of the current device
	public abstract int baudRate();
	
	public static JSONObject translatePeelMessageToJObject(String message) {
		return new JSONObject();
	}

	// Peel is called once. user should read as many messages as possible
	// returns the number of bytes read; or 1 on failure
	// start is the offset into the array 
	public abstract int peel(byte[] buffer, int start, int bytes_available);
	
	private class iothread implements Runnable {
		int asleep_time = (1000 / 60);
		
		@Override
		public void run() {
			while(running) {
				try {
					num_bytes += serial_input.read(buf, start + num_bytes, buf.length - num_bytes); 
					//System.out.printf(new String(buf));
					//System.out.printf("%d\n", bytes);
				} catch (IOException e) {
					// TODO handle this error reasonably.
					e.printStackTrace();
				}
				
				while(true) {
					int num_read = peel(buf, start, num_bytes);
					if(num_read == 0) break;
					start += num_read;
					num_bytes -= num_read;
				}
				
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