package com.roboclub.robobuggy.serial;

/**
 * Class representing a robobuggy serial message
 */
public class RBSerialMessage 
{

	public static final byte ENC_TICKS_SINCE_LAST = (byte)0;
	public static final byte ENC_TICK_SINCE_RESET = (byte)1;
	public static final byte ENC_MS_SINCE_RESET = (byte)2;

	public static final byte RBSM_MID_MEGA_BRAKE_COMMAND = (byte)18;
	public static final byte RBSM_MID_MEGA_STEER_COMMAND = (byte)20;
	public static final byte BRAKE = (byte)21;
	public static final byte AUTO = (byte)22;
	public static final byte BATTERY = (byte)23;
	public static final byte RBSM_MID_MEGA_STEER_FEEDBACK = (byte)24; //potentiometer
	public static final byte RBSM_MID_MEGA_STEER_ANGLE = (byte)20; //Arduino to servo
	public static final byte FP_HASH = (byte)30;
	
	public static final byte LIGHTING_ID = (byte)50;
	
	public static final byte RBSM_MID_RESERVED = (byte)252;
	public static final byte RBSM_MID_ERROR = (byte)254;
	public static final byte RBSM_MID_DEVICE_ID = (byte)255;
	
	public static final byte ERROR = (byte) -2;
	public static final byte DEVICE_ID = (byte) -1;

	public static final byte FOOTER = (byte)0x0A;

	private byte headerByte;
	private int dataBytes;
	
	/**
	 * Construct a new {@link RBSerialMessage} object
	 * @param header byte of the RBSM message header
	 * @param data 4 bytes of the RBSM message payload
	 */
	public RBSerialMessage(byte header, int data) 
	{
		headerByte = header;
		dataBytes = data;
	}

	/**
	 * Returns the header byte of the {@link RBSerialMessage}
	 * @return the header byte of the {@link RBSerialMessage}
	 */
	public byte getHeaderByte() 
	{
		return headerByte;
	}

	/**
	 * Returns the payload bytes of the {@link RBSerialMessage}
	 * @return the payload bytes of the {@link RBSerialMessage}
	 */
	public int getDataWord() 
	{
		return dataBytes;
	}
	
	/**
	 * Determines if the headerByte is a valid RBSM header
	 * @param headerByte header byte
	 * @return true iff the headerByte is valid
	 */
	public static boolean isValidHeader(byte headerByte) 
	{
		switch (headerByte) 
		{
			default:
				return false; 
			case ENC_MS_SINCE_RESET:
			case ENC_TICK_SINCE_RESET:
			case ENC_TICKS_SINCE_LAST:
				
			case RBSM_MID_MEGA_STEER_FEEDBACK:
			case BRAKE:
			case AUTO:
			case BATTERY:
	//		case RBSM_MID_MEGA_STEER_FEEDBACK:
			case RBSM_MID_MEGA_STEER_COMMAND:
			case FP_HASH:
				
			case ERROR:
			case DEVICE_ID:
				return true;
		}
	}
}
