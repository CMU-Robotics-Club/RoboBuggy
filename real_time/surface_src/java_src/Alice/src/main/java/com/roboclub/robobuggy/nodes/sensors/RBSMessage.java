package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.serial.RBSerialMessage;

/**
 * Private class used to represent RBSM messages
 */
public class RBSMessage {
	private static final byte HEADER = RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_COMMAND");
	private static final byte FOOTER = RBSerialMessage.getHeaderByte("FOOTER");

	private short angle;
	private boolean brakesEngaged;

	/**
	 * Create a new RBSMessage object to send to the Arduino
	 * @param angle desired commanded angle in degrees*1000
	 * @param brakesEngaged desired brake state
	 */
	public RBSMessage(short angle, boolean brakesEngaged) {
		this.angle = angle;
		this.brakesEngaged = brakesEngaged;
	}

	/**
	 * Returns a byte array of the RBSMessage to send to the Arduino
	 * @return a byte array of the RBSMessage to send to the Arduino
	 */
	public byte[] getMessageBytes() {
		byte[] bytes = new byte[6];
		bytes[0] = HEADER;
		bytes[1] = (byte)((angle >> 8)&0xFF);
		bytes[2] = (byte)(angle);
		if(brakesEngaged)
			bytes[3] = (byte)1;
		else
			bytes[3] = (byte)0;
		bytes[4] = (byte)(0xc0);
		bytes[5] = FOOTER;
		return bytes;
	}
}
