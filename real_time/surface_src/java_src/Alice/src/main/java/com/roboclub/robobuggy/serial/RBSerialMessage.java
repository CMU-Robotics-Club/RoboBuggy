package com.roboclub.robobuggy.serial;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.nodes.sensors.RBSMConfigReader;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Map;
import java.util.Scanner;

/**
 * Class representing a robobuggy serial message
 */
public class RBSerialMessage
{

	
	private int headerNumber;
	private int dataBytes;


	/**
	 * @param headerName the name of the header as it appears in the text file
	 * @return the byte for that header name
	 */
	public static  synchronized byte getHeaderByte(String headerName) {
		return RBSMConfigReader.getInstance().getHeaders().get(headerName).getAsByte();
	}

	/**
	 * Construct a new {@link RBSerialMessage} object
	 * @param headerName byte of the RBSM message header
	 * @param data 4 bytes of the RBSM message payload
	 */
	public RBSerialMessage(String headerName, int data)
	{
		headerNumber = RBSMConfigReader.getInstance().getHeaders().get(headerName).getAsByte();
		dataBytes = data;
	}

	/**
	 * Constructs a direct {@link RBSerialMessage} object
	 * Rather than use the existing headers, this is meant for constructing from peel
	 * @param header the direct header
	 * @param data the data
	 */
	public RBSerialMessage(byte header, int data) {
		headerNumber = header;
		dataBytes = data;
	}

	/**
	 * Returns the header int of the {@link RBSerialMessage}
	 * @return the header int of the {@link RBSerialMessage}
	 */
	public int getHeaderNumber()
	{
		return headerNumber;
	}

	/**
	 * Returns the payload bytes of the {@link RBSerialMessage}
	 * @return the payload bytes of the {@link RBSerialMessage}
	 */
	public int getDataWord()
	{
		return dataBytes;
	}


}
