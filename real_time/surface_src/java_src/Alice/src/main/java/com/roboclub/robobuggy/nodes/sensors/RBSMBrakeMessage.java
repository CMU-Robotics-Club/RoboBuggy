package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.serial.RBSerialMessage;

/**
 * Class used to represent RBSM brake messages
 */
public class RBSMBrakeMessage {
	private static final byte HEADER = RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_BRAKE_COMMAND");
	private static final byte FOOTER = RBSerialMessage.getHeaderByte("FOOTER");

	private boolean brakesEngaged;

	/**
	 * Create a new {@link RBSMBrakeMessage} object to send to the Arduino
	 * @param brakesEngaged desired brake state
	 */
	public RBSMBrakeMessage(boolean brakesEngaged) {
		this.brakesEngaged = brakesEngaged;
	}

	/**
	 * Returns a byte array of the {@link RBSMBrakeMessage} to send to the Arduino
	 * @return a byte array of the {@link RBSMBrakeMessage} to send to the Arduino
	 */
	public byte[] getMessageBytes() {
		byte[] bytes = new byte[6];
		bytes[0] = HEADER;
		bytes[1] = (byte)0x00;
		bytes[2] = (byte)0x00;
		bytes[3] = (byte)0x00;
		if(brakesEngaged)
			bytes[4] = (byte)0x01;
		else
			bytes[4] = (byte)0x00;
		bytes[5] = FOOTER;
		return bytes;
	}
}
