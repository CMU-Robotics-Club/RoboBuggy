package com.roboclub.robobuggy.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.Handler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;
import java.util.logging.StreamHandler;

import com.roboclub.robobuggy.ui.Gui;

/**
 * Logs data from the sensors
 * 
 * @author Joe Doyle  && Trevor Decker
 */
public final class RobotLogger {
	public final Logger message;
	public final SensorLogger sensor;
	private static RobotLogger instance;
	private static File logDir;
	
	public static RobotLogger getInstance(){
		if(instance == null){
			logDir = new File("C:\\Users\\abc\\buggy-log");
			
			if (!logDir.exists()) {
				logDir.mkdirs();
				System.out.println("Created directory: " + logDir.getAbsolutePath());
			}
				try {
					instance = new RobotLogger(logDir);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
		}
		return instance;
		
	}
	
	public static void CloseLog() {
		if (instance != null) {
			Handler[] handlers = instance.message.getHandlers();
			
			for (int i = 0; i < handlers.length; i++) {
				handlers[i].close();
				instance.message.removeHandler(handlers[i]);
			}
		}
	}
	
	public static void CreateLog() {
		getInstance();
		
		if (instance != null) {
			SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
			File msgFile = new File(logDir,df.format( new Date()) + "-messages.log");
			
			try {
				msgFile.createNewFile();
				//TODO fix Gui.UpdateLogName(msgFile.getName());
				
				Handler handler = new StreamHandler(new FileOutputStream(msgFile),
						new SimpleFormatter());
				instance.message.addHandler(handler);
				System.out.println("Created Log File: " + msgFile.getName());
			} catch (FileNotFoundException e) {
				e.printStackTrace();
				throw new RuntimeException("Could not open message log file (" + msgFile + ")!");
			} catch (IOException e) {
				e.printStackTrace();
				throw new RuntimeException("Could not create new file (" + msgFile + ")!");
			}
		}
	}
	
	
	private RobotLogger(File logdir) throws Exception {
		this.message = Logger.getLogger("RoboBuggy");
		this.sensor = new SensorLogger(logdir,new Date());
	}
}
