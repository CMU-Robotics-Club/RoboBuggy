package com.roboclub.robobuggy.serial;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Map;
import java.util.Scanner;

/**
 * Class representing a robobuggy serial message
 */
public class RBSerialMessage
{
	private static JsonObject headers;
	private static RBSerialMessage instance; 
	
	private byte headerByte;
	private int dataBytes;

	/**
	 * reads the headers text file and puts it into the headers object
	 * @return whether initialization succeeded or not
	 */
	public static synchronized boolean initializeHeaders() {

		if (headers == null){
			instance = new RBSerialMessage();
		}
		return true;

	}
	
	private  RBSerialMessage(){

		headers = new JsonObject();

		try {
			Scanner fileIn = new Scanner(new File(RobobuggyConfigFile.RBSM_HEADER_FILE_LOCATION), "UTF-8");
			while (fileIn.hasNextLine()) {
				String line = fileIn.nextLine();
				if (!line.equals("")) {
					String[] lineContents = line.split(", ");
					String headerName = lineContents[0];
					byte headerByte = Byte.parseByte(lineContents[1]);

					headers.addProperty(headerName, headerByte);
				}
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @param headerName the name of the header as it appears in the text file
	 * @return the byte for that header name
	 */
	public static  synchronized byte getHeaderByte(String headerName) {
		return headers.get(headerName).getAsByte();
	}

	/**
	 * Construct a new {@link RBSerialMessage} object
	 * @param headerName byte of the RBSM message header
	 * @param data 4 bytes of the RBSM message payload
	 */
	public RBSerialMessage(String headerName, int data)
	{
		headerByte = headers.get(headerName).getAsByte();
		dataBytes = data;
	}

	/**
	 * Constructs a direct {@link RBSerialMessage} object
	 * Rather than use the existing headers, this is meant for constructing from peel
	 * @param header the direct header
	 * @param data the data
	 */
	public RBSerialMessage(byte header, int data) {
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

		//see if that's a value in the headers object
		for (Map.Entry<String, JsonElement> object : headers.entrySet()) {
			if (object.getValue().getAsByte() == headerByte) {
				return true;
			}
		}

		return false;
	}
}
