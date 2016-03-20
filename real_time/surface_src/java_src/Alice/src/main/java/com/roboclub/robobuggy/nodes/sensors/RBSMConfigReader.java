package com.roboclub.robobuggy.nodes.sensors;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Map;
import java.util.Scanner;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.serial.RBSerialMessage;

public class RBSMConfigReader {
	private static JsonObject headers;
	private static RBSMConfigReader instance;
	
	public static RBSMConfigReader getInstance(){
		if(instance == null){
			instance = new RBSMConfigReader();
		}
		return instance;
	}
	
	public JsonObject getHeaders(){
		return headers;
	}
	
	public RBSMConfigReader() {
		headers = new JsonObject();
		try {
			Scanner fileIn = new Scanner(new File(RobobuggyConfigFile.RBSM_HEADER_FILE_LOCATION), "UTF-8");
			while (fileIn.hasNextLine()) {
				String line = fileIn.nextLine();
				if (!line.equals("")) {
					String[] lineContents = line.split(", ");
					String headerName = lineContents[0];
					int headerNumber = Integer.parseInt(lineContents[1]);

					headers.addProperty(headerName, headerNumber);
				}
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
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
