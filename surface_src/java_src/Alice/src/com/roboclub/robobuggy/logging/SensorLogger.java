package com.roboclub.robobuggy.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * Logs data from the sensors
 * 
 * @author Joe Doyle
 * @author Trevor Decker
 */
public final class SensorLogger {
	private final PrintStream _log;
	private final Queue<String> _logQueue;
	private final ArrayList<Subscriber> subscribers;

	private static final Queue<String> startLoggingThread(PrintStream stream) {
		final LinkedBlockingQueue<String> ret = new LinkedBlockingQueue<>();
		// TODO
		Thread logging_thread = new Thread() {
			public void run() {
				while (true) {
					try {
						String line = ret.take();
						if (line == null) {
							break;
						}
						stream.println(line);
					} catch (InterruptedException e) {
						//TODO add to messages 
						e.printStackTrace();
					}
				}
			}
		};
		// TODO this potentially prevents a problem where the logger gets CPU
		// starved.
		// the actual fix probably involved throttling the sensors to a
		// reasonable update
		// frequency
		logging_thread.setPriority(6);
		logging_thread.start();
		return ret;
	};

	SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");

	public SensorLogger(File outputDir, Date startTime) throws Exception {
		if (outputDir == null) {
			throw new Exception("Output Directory was null!");
		} else if (!outputDir.exists()) {
			outputDir.mkdirs();
		}

		File logFile = new File(outputDir, "sensors.txt");
		System.out.println("FileCreated: " + logFile.getAbsolutePath());
		try {
			_log = new PrintStream(logFile);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException("Cannot create sensor log file ("
					+ logFile + ")!");
		}
		_logQueue = startLoggingThread(_log);

		// Subscribe to ALL THE PUBLISHERS
		subscribers = new ArrayList<Subscriber>();
		for (SensorChannel channel : SensorChannel.values()) {
			subscribers.add(
				new Subscriber(channel.getMsgPath(), new MessageListener() {
					@Override
					public void actionPerformed(String topicName, Message m) {
						_logQueue.offer(topicName + "," + m.toLogString());
					}
				}));
		}
	}
}
