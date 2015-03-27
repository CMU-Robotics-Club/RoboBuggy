package com.roboclub.robobuggy.serial;

public class RBSerialMessage {

	
	public static final byte ENC_RESET = (byte)0;
	public static final byte ENC_TICK = (byte)1;
	public static final byte ENC_TIME = (byte)2;
	
	public static final byte STEERING = (byte)20;
	public static final byte BRAKE = (byte)21;
	public static final byte AUTO = (byte)22;
	public static final byte BATTERY = (byte)23;
	
	public static final byte ERROR = (byte)254;
	public static final byte DEVICE_ID = (byte)255;


	private byte header_byte;
	private int data_bytes;
	
	public RBSerialMessage(byte header, int data) {
		header_byte = header;
		data_bytes = data;
	}

	public byte getHeaderByte() {
		return header_byte;
	}

	public int getDataWord() {
		return data_bytes;
	}
	
	public static boolean isValidHeader(byte headerByte) {
		switch (headerByte) {
			default:
				return false; 
			case ENC_RESET:
			case ENC_TIME:
			case ENC_TICK:

			case STEERING:
			case BRAKE:
			case AUTO:
			case BATTERY:
				
			case ERROR:
			case DEVICE_ID:
				return true;
		}
	}
}
