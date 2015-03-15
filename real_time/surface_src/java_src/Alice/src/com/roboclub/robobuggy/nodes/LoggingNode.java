package com.roboclub.robobuggy.nodes;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Subscriber;

public class LoggingNode implements Node {

	// Open the file
	FileOutputStream outputFile = null;
	Subscriber s;
	// Get the folder that we're going to use

	// TODO get folder name from file.
	public LoggingNode(String topicName, String directoryPath) {
	
		try {
			new File(directoryPath).mkdirs();
			File tmp = new File(directoryPath + "\\" + topicName + ".txt");
			tmp.createNewFile();
			outputFile = new FileOutputStream(tmp);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			System.out.printf("file is screwed");
			return;
		} catch (IOException e) {
			System.out.printf("file is screwed");
			e.printStackTrace();
		}

		// Start the subscriber
		s = new Subscriber(topicName, new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				try {
					outputFile.write(m.toLogString().getBytes());
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
