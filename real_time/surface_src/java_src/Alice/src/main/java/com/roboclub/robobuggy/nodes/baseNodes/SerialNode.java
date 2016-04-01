package com.roboclub.robobuggy.nodes.baseNodes;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;


/**
 * Abstract class extended to create a decorator node that uses 
 * serial communications
 */
public abstract class SerialNode extends BuggyDecoratorNode {
	
	private static final int TIMEOUT = 2000;
	//Need to manage real, simulated, and calculated sensors
	
	// Initialize first
	private String threadName;
	private SerialPort sp;

	// Initialize after getting a serial port
	private InputStream serialInput;
	private OutputStream serialOutput;
	
	private Thread ioThread;
	
	private boolean running = false;
	private byte[] buf = new byte[128];
	private int start = 0;
	private int numBytes = 0;

	/**
	 * Creates a {@link SerialNode} decorator for the specified {@link BuggyNode}
	 * @param base {@link BuggyNode} to decorate
	 * @param threadName name of the thread
	 * @param portName name of the desired serial port
	 * @param baudRate baud rate of the serial port
	 */
	public SerialNode(BuggyNode base, String threadName, String portName,
			int baudRate) {
		super(base, portName);
		this.setName(threadName);
		this.threadName = threadName;
		this.sp = connect(portName, baudRate);
    }
	
	// Open a serial port
	// Returns null if unable to connect, otherwise SerialPort
	private static SerialPort connect(String portName, int baudRate) {
	        CommPortIdentifier portIdentifier;
			try {
				portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
	        if ( portIdentifier.isCurrentlyOwned() ) {
	        	new RobobuggyLogicNotification("Error: Port Currently in use:"+portIdentifier.getName(), RobobuggyMessageLevel.EXCEPTION);
	        } else { 
	            CommPort commPort = portIdentifier.open(portName, TIMEOUT);
	            if ( commPort instanceof SerialPort ) {
	                SerialPort serialPort = (SerialPort) commPort;
	                serialPort.setSerialPortParams(baudRate,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
	                return serialPort;
	            }
	        }
			} catch (NoSuchPortException | PortInUseException | UnsupportedCommOperationException e) {
				new RobobuggyLogicNotification("Error: Unable to connect to port"+portName, RobobuggyMessageLevel.EXCEPTION);

			}		
		return null;
    }

	/**
	 * Call to send bytes over the serial port
	 * @param bytes byte array to send
	 * @return true iff the bytes are transmitted successfully
	 */
	public boolean send(byte[] bytes) {


		if(serialOutput == null) {
			return false;
		}
	
		try {
			serialOutput.write(bytes);
			serialOutput.flush();
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
	protected boolean startDecoratorNode() {

		// Set port to be the right baud rate
		int baudRate = getBaudRate();
		try {
			sp.setSerialPortParams(baudRate,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
		} catch (UnsupportedCommOperationException e) {
			// TODO Auto-generated catch block
			new RobobuggyLogicNotification("Unsupported communication operation over serial port."+e.getMessage(), RobobuggyMessageLevel.EXCEPTION);
			return false;
		} catch (NullPointerException e) {
			new RobobuggyLogicNotification("Null Pointer Exception, unable to initialize the serial port. "+
		e.getMessage(), RobobuggyMessageLevel.EXCEPTION);
			return false;
		}

		// Set port to be non-blocking....maybe.
		sp.disableReceiveTimeout();
		sp.disableReceiveThreshold();
		// Set port to have a reasonable input buffer size?

		try {
			serialInput = sp.getInputStream();
			serialOutput = sp.getOutputStream();
		} catch (IOException e) {
			new RobobuggyLogicNotification("serial broken", RobobuggyMessageLevel.EXCEPTION);
			return false;
		}

		//Set the node to running
		running = true;
		// Begin the madness
		ioThread = new Thread(new IoThread(), threadName + "-serial");
		ioThread.setPriority(Thread.MAX_PRIORITY);
		ioThread.start();
		
		return true;
	}

	/**
	 * Shuts down the Serial reader thread
	 * {@inheritDoc}*/
	@Override
	protected boolean shutdownDecoratorNode() {
		running = false;
		// iothread will commence shutdown next loop.
	
		// Wait forever to join...hope that we're not seized up...
		if (ioThread != null) {
			try {
				ioThread.join();
			} catch (InterruptedException e) {
				e.printStackTrace();
				return false;
			}
		}
		return true;
	}

	/**
	 * Returns true iff the data type is readable by this current node
	 * @param sample byte array data
	 * @return true iff the data type is readable by this current node
	 */
	public abstract boolean matchDataSample(byte[] sample);

	/**
	 * Returns the number of bytes needed to match 
	 * @return the number of bytes needed to match
	 */
	public abstract int matchDataMinSize();

	/**
	 * Returns the expected baud rate of the current device
	 * @return the expected baud rate of the current device
	 */
	public abstract int getBaudRate();
	

	/**
	 * Peel is called once. User should read as many messages as possible
	 * @param buffer byte array read from the serial port
	 * @param start offset into buffer
	 * @param bytesAvailable number of bytes available to read
	 * @return the number of bytes read, or 1 on failure
	 */ 
	public abstract int peel(byte[] buffer, int start, int bytesAvailable);
	
	/**
	 * Private class used as a thread to read serial messages
	 */
	private class IoThread implements Runnable {
		private int asleepTime = (1000 / 60);
		
		@Override
		public void run() {
			while(running) {

				try {
					numBytes += serialInput.read(buf, start + numBytes, buf.length - numBytes); 

					while(true) 
					{
						int numRead = peel(buf, start, numBytes);
						if(numRead == 0)
						{
							break;
						}
						start += numRead;
						numBytes -= numRead;
					}
					
					// Shift the array by the amount that we read.
					// TODO this is stupid and should be fixed
					for(int i = 0; i < numBytes; i++) {
						buf[i] = buf[start+i];
					}
					start = 0;
				
					try {
						Thread.sleep(asleepTime);
					} catch (InterruptedException e) {
						new RobobuggyLogicNotification("sleep .... interrupted?", RobobuggyMessageLevel.EXCEPTION);
					}
				} catch (IOException e) {
					// TODO handle this error reasonably.
					setNodeState(NodeState.DISCONNECTED);
					e.printStackTrace();
				}
			}
		}			
	}
}