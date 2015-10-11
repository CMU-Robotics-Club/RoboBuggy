package com.roboclub.robobuggy.serial;

public class RBSerialMessage {

	
	public static final byte ENC_TICKS_SINCE_LAST = (byte)0;
	public static final byte ENC_TICK_SINCE_RESET = (byte)1;
	public static final byte ENC_MS_SINCE_RESET = (byte)2;
	
	
	
	public static final byte STEERING = (byte)20;
	public static final byte BRAKE = (byte)21;
	public static final byte AUTO = (byte)22;
	public static final byte BATTERY = (byte)23;
	public static final byte RBSM_MID_MEGA_STEER_FEEDBACK = (byte)24; //potentiometer 

	public static final byte FP_HASH = (byte)30;
	
	public static final byte LIGHTING_ID = (byte)50;
	
	public static final byte RBSM_MID_RESERVED = (byte)252;
	public static final byte RBSM_MID_ERROR = (byte)254;
	public static final byte RBSM_MID_DEVICE_ID = (byte)255;
	
	public static final byte ERROR = (byte) -2;
	public static final byte DEVICE_ID = (byte) -1;
	//public static final byte ERROR = (byte)254;
	//public static final byte DEVICE_ID = (byte)255;


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
			case ENC_MS_SINCE_RESET:
			case ENC_TICK_SINCE_RESET:
			case ENC_TICKS_SINCE_LAST:
				
			case STEERING:
			case BRAKE:
			case AUTO:
			case BATTERY:
			case RBSM_MID_MEGA_STEER_FEEDBACK:

			case FP_HASH:
				
			case ERROR:
			case DEVICE_ID:
				return true;
		}
	}
}
