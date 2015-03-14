package com.roboclub.robobuggy.nodes;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Subscriber;

public class LoggingNode implements Node {

	
	Subscriber s;
	ZipOutputStream zipOutputFile;
	// Get the folder that we're going to use

	// TODO get folder name from file.
	String loggingDirectory = "C:\\Users\\Matt\\buggy-log";
	
	public LoggingNode(String topicName) {
		// Open the file
		FileOutputStream outputFile;
		try {
			outputFile = new FileOutputStream(loggingDirectory+"\\directory.zip");
			BufferedOutputStream bOutputFile = new BufferedOutputStream(outputFile);
			zipOutputFile = new ZipOutputStream(bOutputFile);
			ZipEntry ze = new ZipEntry("first.txt");
			zipOutputFile.putNextEntry(ze);
			
			byte[] barr = {51, 52, 53};
			zipOutputFile.write(barr, 0, 3);
			zipOutputFile.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Start the subscriber
		s = new Subscriber(topicName, new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				try {
					zipOutputFile.write(m.toLogString().getBytes());
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
		
		// Attempt to write immediately to the zip file
		
	}
	
	@Override
	public boolean shutdown() {
		// TODO Actually shutdown stuff.
		return false;
	}

}
