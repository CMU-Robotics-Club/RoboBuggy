package com.roboclub.robobuggy.logging.autoLogging;

import java.io.File;
import java.io.IOException;

import com.roboclub.robobuggy.main.RobobuggyLogicException;
import com.roboclub.robobuggy.ros.SensorChannel;

//Future todo handle sub folders properly  
//Future todo handle log folder location being moved 
//TODO integrate into Alice 

public class testDrive {
	static ServerCommunication test;

	public static void main(String[] args) {
		RobobuggyLogicException.setupLogicException(SensorChannel.LOGIC_EXCEPTION);
		
		File whereToSave = new File("LOG_FILES");
		String DriveStorageFolder_id ="0B1IjfVrCn6dNZjZfems2ZUlXNlE";
		try {
			autoLogging autoLogger  = autoLogging.startAutoLogger(whereToSave, DriveStorageFolder_id);
			int count = 6;
			File nFolder = new File("LOG_FILES/test"+count);
			autoLogger.startTrackingLog(nFolder);
			System.out.println(autoLogger.getNumLogs());
			autoLogger.saveLogDataToFolders();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		/*
		try {
			final URL url = new URL("http://www.google.com");
			url.openConnection();
			ServerCommunication drive = ServerCommunication.getInstance();
			final java.io.File UPLOAD_FILE = new java.io.File("LOG_FILES");
			try {
				drive.addFolderToDrive(UPLOAD_FILE);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} catch (MalformedURLException e1) {
			System.out.println("Can not connect to internet will try again later");
			e1.printStackTrace();
		} catch (IOException e) {
			System.out.println("Can not connect to internet will try again later");
			e.printStackTrace();
		}  
		*/
		
	}

}
