package com.roboclub.robobuggy.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.LinkedBlockingQueue;

import com.roboclub.robobuggy.ros.Message;

/**
 * Logs data to a file.
 *
 * TODO: Error conditions/What to do when we have an exception should be
 * reviewed.
 *
 * @author Joe Doyle
 * @author Matt Sebek (sebek.matt@gmail.com)
 */
public final class MessageLogWriter {
	private PrintStream csvOutstream = null;
	private LinkedBlockingQueue<String> lineQueue = new LinkedBlockingQueue<String>();

	// Used for directory names
	private SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");

	// returns the name of the logger.
	private String getFileName(Date startTime) {
		String outputFileName = startTime.toString();
		outputFileName = outputFileName.replaceAll(" ", "");
		outputFileName = outputFileName.replaceAll(":", "_");
		return outputFileName;
	}

	/**
	 * Constructs a new {@link MessageLogWriter}
	 * @param outputDir {@link File} of the output directory
	 * @param startTime {@link Date} of the starting time
	 */
	public MessageLogWriter(File outputDir, Date startTime) {
		String filename = getFileName(startTime);
		init(outputDir, filename, startTime);
	}

	/**
	 * Constructs a new {@link MessageLogWriter}
	 * @param outputDir {@link File} of the output directory
	 * @param logFileName name of the output log file
	 * @param startTime {@link Date} of the starting time
	 */
	public MessageLogWriter(File outputDir, String logFileName, Date startTime) {
		init(outputDir, logFileName, startTime);
	}

	// TODO think about error cases more
	private void init(File outputDir, String outputFilename, Date startTime) {
		// Create output directory
		if (outputDir == null) {
			throw new RuntimeException("Output Directory was null!");
		} else if (!outputDir.exists()) {
			outputDir.mkdirs();
		}

		File csvFile = new File(outputDir, outputFilename + "sensors.csv");
		System.out.println("Logfile Created: " + outputFilename);

		// TODO fix Gui.UpdateLogName( outputFileName );
		try {
			csvOutstream = new PrintStream(csvFile);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException("Cannot create sensor log file ("
					+ csvFile + ")!");
		}

		// Spin up loggin thread
		new Thread(new CsvWriter(csvOutstream)).start();

	}

	// File imgdir = new File(outputDir, df.format(startTime) + "-images");
	// imgdir.mkdirs();
	// _imgQueue = startImgThread(imgdir);

	/*
	 * ArrayList<String> keys = new ArrayList<>(); keys.add("Timestamp"); for
	 * (String[] ks : _keySets) { for (String k : ks) { keys.add(k); } } _keys =
	 * new String[keys.size()]; keys.toArray(_keys); _csvQueue.offer(_keys); }
	 */

	/**
	 * Runnable that writes to a CSV file-stream continuously
	 */
	private class CsvWriter implements Runnable {
		private PrintStream ps;

		/**
		 * Construct a new {@link CsvWriter} object
		 * @param stream {@link PrintStream} to write to
		 */
		public CsvWriter(PrintStream stream) {
			ps = stream;
		}

		/**
		 * Constantly write received messages to the log file
		 */
		@Override
		public void run() {
			while (true) {
				try {
					String line = lineQueue.take();
					ps.write(line.getBytes());
				} catch (InterruptedException e) {
					e.printStackTrace();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
	}

	/**
	 * Submit a {@link Message} to be logged by the {@link MessageLogWriter}
	 * @param m {@link Message} to be logged
	 */
	public void log(Message m) {
		lineQueue.offer(m.toLogString());
	}
}