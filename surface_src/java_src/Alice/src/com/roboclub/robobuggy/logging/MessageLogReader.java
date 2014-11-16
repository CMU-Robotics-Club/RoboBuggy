package com.roboclub.robobuggy.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.LinkedList;
import java.util.List;

import com.roboclub.robobuggy.ros.Message;

/**
 * Read the data from the file.
 *
 * TODO: Error conditions/What to do when we have an exception should be
 * reviewed.
 *
 * @author Joe Doyle
 * @author Matt Sebek (sebek.matt@gmail.com)
 */
public final class MessageLogReader {
	// Used for directory names
	private SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");

	// Loop through each type of message, and see if it fits the bill. gT
	private loadMessageFromString(String loggedMessage) {
		if()


	}

	// returns the name of the logger.
	public static List<Message> getAllMessages(File logFile) {
		if (!logFile.isFile()) {
			throw new RuntimeException("Log File was not able to be read!");
		}

		// TODO for optimal performance, make this lazy/streams based

		// Begin reading lines from the file
		FileReader r = new FileReader(logFile);

		while(true) {
			String logFileLine = "";


		}


		// Convert to messages
		for()
		return new LinkedList<Message>();
	}

	private void init(File outputDir, String outputFilename, Date startTime) {
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

	// Runnable that writes to a CSV file-stream continuously
	class csv_writer implements Runnable {
		PrintStream ps;

		public csv_writer(PrintStream stream) {
			ps = stream;
		}

		@Override
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

	public void log(String message) {
		_line_queue.offer(message);
	}
}
