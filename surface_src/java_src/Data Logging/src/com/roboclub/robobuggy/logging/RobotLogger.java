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

import javax.swing.text.DateFormatter;

/**
 * Logs data from the sensors
 * 
 * @author Joe Doyle
 */
public final class RobotLogger {
	public final Logger message;
	public final SensorLogger sensor;
	private static RobotLogger instance = null;
	
	public static RobotLogger getInstance(){
		if(instance == null){
			File fileToLogTo = new File("C:\\Users\\Matt\\buggy-log");
			fileToLogTo.mkdirs();
			instance = new RobotLogger(fileToLogTo);
		}
		return instance;
		
	}

	SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
	private RobotLogger(File logdir) {
		Date d = new Date();
		this.message = Logger.getLogger("RoboBuggy");
		File msgFile = new File(logdir,df.format(d) + "-messages.log");
		Handler handler = null;
		try {
			msgFile.createNewFile();
			handler = new StreamHandler(new FileOutputStream(msgFile),
												new SimpleFormatter());
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException("Could not open message log file (" + msgFile + ")!");
		} catch (IOException e) {
			e.printStackTrace();
			throw new RuntimeException("Could not create new file (" + msgFile + ")!");
		}
		this.message.addHandler(handler);
		this.sensor = new SensorLogger(logdir,new Date());
	}
	
	private RobotLogger _instance;

}
