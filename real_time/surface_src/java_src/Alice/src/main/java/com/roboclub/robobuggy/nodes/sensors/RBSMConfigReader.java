package com.roboclub.robobuggy.nodes.sensors;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Map;
import java.util.Scanner;

/**
 * 
 * @author Trevor Decker
 * An object for reading the header configuration file so that low level and high level are synchronized on the RBSM communication protocol
 */
public class RBSMConfigReader {
	private  JsonObject headers;
	private static RBSMConfigReader instance;
	
	/**
	 * evaluates to a reference of the only RBSMConfigReader on the system
	 * allows for any object to access header information  
	 * @return RBSMConfigReader reference
	 */
	public static synchronized RBSMConfigReader getInstance(){
		if(instance == null){
			instance = new RBSMConfigReader();
		}
		return instance;
	}
	
	/**
	 * gets the headers json object  
	 * @return the json object to lookup header values
	 */
	public  JsonObject getHeaders(){
		return headers;
	}
	
	/**
	 * The constructor for the RBSMConfig reader, this file function is where the RBSM header file is read and its data is stored
	 */
	public RBSMConfigReader() {
		headers = new JsonObject();
		try {
			Scanner fileIn = new Scanner(new File(RobobuggyConfigFile.RBSM_HEADER_FILE_LOCATION), "UTF-8");
			while (fileIn.hasNextLine()) {
				String line = fileIn.nextLine();
				if (!line.equals("") && !line.contains("//")) {
					String[] lineContents = line.split(", ");
					String headerName = lineContents[0];
					int headerNumber = Integer.parseInt(lineContents[1]);

					headers.addProperty(headerName, headerNumber);
				}
			}
		} catch (FileNotFoundException e) {
			new RobobuggyLogicNotification("rbsm header File not read correctly: "+e.getMessage(), RobobuggyMessageLevel.EXCEPTION);
		}
	}
	
	/**
	 * Determines if the headerByte is a valid RBSM header
	 * @param headerByte header byte
	 * @return true iff the headerByte is valid
	 */
	public  boolean isValidHeader(byte headerByte)
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
