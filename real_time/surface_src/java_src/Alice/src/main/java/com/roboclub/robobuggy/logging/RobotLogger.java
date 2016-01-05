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
import com.roboclub.robobuggy.main.config;

/**
 * Logs data from the sensors
 * 
 * @author Joe Doyle 
 * @author Trevor Decker
 */
public final class RobotLogger {
	private static Logger message;
	private static SensorLogger sensor;
	private static RobotLogger instance;
	private static File logDir;  //TODO describe the diffrence between LogDir and  logFolder 
	private static File logFolder;

	/**
	 * Returns a reference to the one {@link RobotLogger} object.
	 * If no {@link RobotLogger} object exists, one will be constructed.
	 * @return a reference to the one {@link RobotLogger} object
	 */
	public static RobotLogger getInstance() {
		if (instance == null) {
			logDir = new File(config.LOG_FILE_LOCATION);
			
			logDir.mkdirs();
			
//			if (!logDir.exists()) {
//				logDir.mkdirs();
//				System.out.println("Created directory: "
//						+ logDir.getAbsolutePath());
//			}
			try {
				instance = new RobotLogger(logDir);
			} catch (Exception e) {
				e.printStackTrace();
			}

		}
		return instance;

	}

	/**
	 * Closes the {@link RobotLogger} log file
	 */
	public static void closeLog() {
		if (instance != null) {
			Handler[] handlers = instance.message.getHandlers();

			for (int i = 0; i < handlers.length; i++) {
				handlers[i].close();
				instance.message.removeHandler(handlers[i]);
			}
		}
	}

	/**
	 * Closes the current log and creates a new one
	 */
	public void startNewLog() {
		// TODO send signal to vision to start a new log also
		closeLog();
		createLog();
	}

	/**
	 * Creates a new log file to record data in using the {@link RobotLegger}
	 */
	public static void createLog() {
		getInstance();

		if (instance != null) {
			SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
			logFolder = new File(logDir, df.format(new Date()));
			logFolder.mkdirs();

			File msgFile = new File(logFolder, "messages.log");
			sensor = new SensorLogger(logFolder, new Date());

			try {
				msgFile.createNewFile();
				// TODO fix Gui.UpdateLogName(msgFile.getName());

				Handler handler = new StreamHandler(new FileOutputStream(
						msgFile), new SimpleFormatter());
				instance.message.addHandler(handler);
				System.out.println("Created Log File: "
						+ msgFile.getAbsolutePath());
			} catch (FileNotFoundException e) {
				e.printStackTrace();
				throw new RuntimeException("Could not open message log file ("
						+ msgFile + ")!");
			} catch (IOException e) {
				e.printStackTrace();
				throw new RuntimeException("Could not create new file ("
						+ msgFile + ")!");
			}
		}
	}

	private RobotLogger(File logdir) throws Exception {
		this.message = Logger.getLogger("RoboBuggy");
		this.sensor = null;
	}
}
