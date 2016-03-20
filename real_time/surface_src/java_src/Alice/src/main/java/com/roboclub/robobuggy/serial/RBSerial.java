package com.roboclub.robobuggy.serial;

import com.roboclub.robobuggy.nodes.sensors.RBSMConfigReader;

/**
 * Class to represent a robobuggy serial connection
 */
public final class RBSerial {
	
	// TODO move this
	public static final int MSG_LEN = 6;
	
	private RBSerial() {}

	/**
	 * Peel is called once. User should read as many messages as possible
	 * @param buf byte array read from the serial port
	 * @param start offset into buffer
	 * @param numElements number of bytes available to read
	 * @return {@link RBPair} with the number of read bytes and the message
	 */ 
	public static RBPair peel(byte[] buf, int start, int numElements) {
		// If there aren't enough bytes, fail immediately
		if(numElements < MSG_LEN) {
			return new RBPair(0, null);
		}
		
		// Peel an ID, or fail
		byte header = buf[start];
		if(!RBSMConfigReader.getInstance().isValidHeader(header)) {
			return new RBPair(1, null);
		}
		if(buf[(start+5) % buf.length] != 0x0A){
			return new RBPair(1, null);
		}
		// Parse an int, or fail
		int payload = parseInt(buf, start, numElements);
		
		return new RBPair(MSG_LEN, new RBSerialMessage(header, payload));
		
	}
	
	private static int parseInt(byte[] buf, int start, int numElements) {
		int val = 0;
		val += (int)buf[(start + 1) % buf.length ] &0xff;
		val = val << 0x8;
		val += (int)buf[(start + 2) % buf.length] &0xff;
		val = val << 0x8;
		val += (int)buf[(start + 3) % buf.length] &0xff;
		val = val << 0x8;
		val += (int)buf[(start + 4) % buf.length] &0xff;
		return val;
	}
	
	
	/*private void something() {
		byte inputBuffer[] = new byte[5];
		int value = parseInt(inputBuffer[1], inputBuffer[2],
				inputBuffer[3], inputBuffer[4]);
		try {
			switch (inputBuffer[0]) {
			case ENC_TIME:
				encTime = value;
				break;
			case ENC_RESET:
				encReset = value;
				break;
			case ENC_TICK:
				encTicks = value;
				estimateVelocity();
				break;
			case ERROR:
				// TODO handle errors
				break;
			}
		} catch (Exception e) {
			System.out.println("Encoder Exception on port: " + this.getName());
			if (this.currState != SensorState.FAULT) {
				this.currState = SensorState.FAULT;
				statePub.publish(new StateMessage(this.currState));
			}
			return;
		}
	}*/
}
