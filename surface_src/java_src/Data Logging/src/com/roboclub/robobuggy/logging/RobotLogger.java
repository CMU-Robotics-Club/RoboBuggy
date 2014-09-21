package com.roboclub.robobuggy.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.Date;
import java.util.logging.Handler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;
import java.util.logging.StreamHandler;

public final class RobotLogger {
	public final Logger message;
	public final SensorLogger sensor;
	
	private RobotLogger(File logdir) {
		Date d = new Date();
		this.message = Logger.getLogger("RoboBuggy");
		File msgFile = new File(logdir,d.toString() + " messages.log");
		Handler handler = null;
		try {
			handler = new StreamHandler(new FileOutputStream(msgFile),
												new SimpleFormatter());
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException("Could not open message log file (" + msgFile + ")!");
		}
		this.message.addHandler(handler);
		this.sensor = new SensorLogger(logdir,new Date());
	}
	
	private RobotLogger _instance;
	public RobotLogger getInstance() {
		// TODO: Give this a real directory to work on
		return _instance == null ? (_instance = new RobotLogger(new File(""))) :
								   _instance;
	}

}
