package org.roboclub.robobuggy.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.LinkedBlockingQueue;

import org.roboclub.robobuggy.ros.Message;

/**
 * Logs data to a file.
 *
 * TODO: Error conditions/What to do when we have an exception should be
 * reviewed.
 *
 * @author Joe Doyle
 * @author Matt Sebek (sebek.matt@gmail.com)
 * @version 0.0
 */
public final class MessageLogWriter {
	private PrintStream _csv_outstream = null;
	private LinkedBlockingQueue<String> _line_queue = new LinkedBlockingQueue<String>();

	// Used for directory names
	private SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");

	/** 
	 * returns the name of the logger.
	 * @param startTime
	 * @return
	 */
	private String getFileName(Date startTime) {
		String outputFileName = startTime.toString();
		outputFileName = outputFileName.replaceAll(" ", "");
		outputFileName = outputFileName.replaceAll(":", "_");
		return outputFileName;
	}

	/**
	 * TODO document
	 * @param outputDir
	 * @param startTime
	 */
	public MessageLogWriter(File outputDir, Date startTime) {
		String filename = getFileName(startTime);
		init(outputDir, filename, startTime);
	}

	/**
	 * TODO document
	 * @param outputDir
	 * @param logFileName
	 * @param startTime
	 */
	public MessageLogWriter(File outputDir, String logFileName, Date startTime) {
		init(outputDir, logFileName, startTime);
	}

	/**
	 * TODO document
	 *  TODO think about error cases more
	 * @param outputDir
	 * @param outputFilename
	 * @param startTime
	 */
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
			_csv_outstream = new PrintStream(csvFile);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException("Cannot create sensor log file ("
					+ csvFile + ")!");
		}

		// Spin up loggin thread
		new Thread(new csv_writer(_csv_outstream)).start();

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
	 *  Runnable that writes to a CSV file-stream continuously
	 *  TODO break out to a separate class
	 */
	class csv_writer implements Runnable {
		PrintStream ps;

		/**
		 * TODO document 
		 * @param stream
		 */
		public csv_writer(PrintStream stream) {
			ps = stream;
		}

		@Override
		/**
		 * TODO document 
		 */
		public void run() {
			while (true) {
				try {
					String line = _line_queue.take();
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
	 * TODO document 
	 * @param m
	 */
	public void log(Message m) {
		_line_queue.offer(m.toLogString());
	}
}