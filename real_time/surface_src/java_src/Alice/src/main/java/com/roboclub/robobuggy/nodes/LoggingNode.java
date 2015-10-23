package com.roboclub.robobuggy.nodes;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.messages.BaseMessage;
import com.roboclub.robobuggy.messages.GuiLoggingButton;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

// When logging begins, a new folder is created, and then logging begins to that folder
public class LoggingNode implements Node {

	String directoryPath;
	String topicName;
	
	BufferedOutputStream outputFile = null;
	
	Subscriber s;
	Subscriber logging_button_sub;
	// Get the folder that we're going to use

	// TODO get folder name from file.
	public LoggingNode(String topicName, final String directoryPath) {
		this.topicName = topicName;
		this.directoryPath  = directoryPath;
		// Start the subscriber
		s = new Subscriber(topicName, new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				if(outputFile == null) {
					return;
				}
				
				try {
					outputFile.write(m.toLogString().getBytes());
					outputFile.write('\n');
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
	
		
		logging_button_sub = new Subscriber(SensorChannel.GUI_LOGGING_BUTTON.getMsgPath(), new MessageListener() {
			@Override 
			public void actionPerformed(String topicName, Message m) {
				GuiLoggingButton glb = (GuiLoggingButton) m;
				switch (glb.lm) {
				case START:
					String d = BaseMessage.format_the_date(glb.timestamp).replace(':', '-'); 
					startLogging(directoryPath + "\\" +  d.replace(" ", "-").replace('/', '\\'));
					break;

				case STOP:
					stopLogging();
					break;
					
				default:
					System.out.println("CANNOT BE HERE NOWW WHYYY");
					return;
				}
				
			}
		});
		
	}

	public void startLogging(String directoryPath) {
		try {
			File tmp = new File(directoryPath + "\\" + topicName + ".txt");
			tmp.getParentFile().mkdirs();
			System.out.println(tmp.getAbsolutePath());
			tmp.createNewFile();
			outputFile = new BufferedOutputStream(new FileOutputStream(tmp));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			System.out.printf("file is screwed");
			return;
		} catch (IOException e) {
			System.out.printf("file is screwed");
			e.printStackTrace();
		}

		
	}

	public void stopLogging() {
		BufferedOutputStream bos = outputFile;
		outputFile = null;
		try {
			bos.close();
		} catch (IOException e) {
			System.out.println("Could not close file properly!");
			e.printStackTrace();
		}
	}
	
	@Override
	public boolean shutdown() {
		// TODO Actually shutdown stuff.
		return false;
	}

	public static JSONObject translatePeelMessageToJObject(String message) {
		// TODO Auto-generated method stub
		return null;
	}
	
}
