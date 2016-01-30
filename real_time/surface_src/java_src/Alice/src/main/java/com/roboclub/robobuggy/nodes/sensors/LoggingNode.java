package com.roboclub.robobuggy.nodes.sensors;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;

import com.google.gson.JsonObject;
import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.ros.*;
import com.roboclub.robobuggy.ui.Gui;

/**
 * {@link SerialNode} for reading in logging commands from the GUI
 * When logging begins, a new folder is created, and then logging begins
 *  to that folder
 */
public class LoggingNode extends BuggyDecoratorNode {

	private Publisher loggingButtonPub;
	private File outputFile;
    private NodeChannel[] filters;
    private LinkedList<Message> messageQueue;
    private LogWriterRunnable loggingThread;
    private boolean keepLogging;

	/**
	 * Create a new {@link LoggingNode} decorator
	 * @param channel the {@link NodeChannel} of the {@link LoggingNode}
     * @param outputDirPath The path to the output directory (not file)
     * @param filters sensors to log. To log all sensors, just use NodeChannel.values()
	 */
	public LoggingNode(NodeChannel channel, String outputDirPath, NodeChannel...filters) {
		super(new BuggyBaseNode(channel));

        this.filters = filters;
        messageQueue = new LinkedList<>();
        keepLogging = true;
        loggingThread = new LogWriterRunnable();

        createNewLogFile(outputDirPath);
        setupSubscriberList();
        startLogging();

    }



    /**
     * Starts the logging process
     */
    private void startLogging() {
        loggingThread.start();
    }

    /**
     * Sets up the subscriber list - Simply enumerates over our NodeChannel filters and adds
     * a subscriber for each one
     */
    private void setupSubscriberList() {
        for (NodeChannel filter : filters) {
            new Subscriber(filter.getMsgPath(), new MessageListener() {
                @Override
                public void actionPerformed(String topicName, Message m) {
                    messageQueue.add(m);
                }
            });
        }
    }


    /**
     * Creates the log file, and returns the status
     * Returns false if anything went wrong, but already throws the logic exception
     *
     * @param outputDirString the directory string of the log file
     * @return the status of the operation - true if it succeeded, false if it didn't
     */
    private boolean createNewLogFile(String outputDirString) {
        File outputDirectory = new File(outputDirString);
        if (!outputDirectory.exists() || !outputDirectory.isDirectory()) {
            new RobobuggyLogicNotification("Output directory path isn't a folder!", RobobuggyMessageLevel.EXCEPTION);
            return false;
        }

        outputFile = new File(outputDirectory.getPath() + "/" + RobobuggyConfigFile.LOG_FILE_NAME);
        try {
            if(!outputFile.createNewFile()) {
                new RobobuggyLogicNotification("Couldn't create log file!", RobobuggyMessageLevel.EXCEPTION);
                return false;
            }
        } catch (IOException e) {
            new RobobuggyLogicNotification("Error reading the filesystem!", RobobuggyMessageLevel.EXCEPTION);
            return false;
        }

        //everything succeeded!
        return true;
    }


	/**{@inheritDoc}*/
	@Override
	protected boolean startDecoratorNode() {
		loggingButtonPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
		
		new Subscriber(Gui.GuiPubSubTopics.GUI_LOG_BUTTON_UPDATED.toString(), new MessageListener() {
			@Override 
			public void actionPerformed(String topicName, Message m) {
				loggingButtonPub.publish(m);
			}
		});
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	protected boolean shutdownDecoratorNode() {
		return true;
	}


    /**
     * LogWriterRunnable - where we actually process each message and write it to the file
     */
    private class LogWriterRunnable extends Thread {

        private int imuHits = 0;
        private int encoderHits = 0;
        private int gpsHits = 0;
        private int brakeHits = 0;
        private int fingerprintHits = 0;
        private int logButtonHits = 0;
        private int steeringHits = 0;
        private int logicNotificationHits = 0;

        private String name = "\"name\": \"Robobuggy Data Logs\",";
        private String schemaVersion = "\"schema_version\": 1.0,";
        private String dateRecorded = "\"date_recorded\": \"" +
                new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS").format(new Date()) + "\",";
        private String swVersion = "\"software_version\": \"" + getCurrentSoftwareVersion() + "\",";
        private String sensorDataHeader = "\"sensor_data\": [";

        @Override
        public synchronized void start() {

            stream.println("{" + "\n    " + name + "\n    " + schemaVersion + "\n    " + dateRecorded
                    + "\n    " + swVersion + "\n    " + sensorDataHeader);

            //always want to log :)
            while (keepLogging) {
                //spin in a loop until a message comes in
                while (messageQueue.isEmpty()) {}
                //now we have a message from the queue
            }
        }


        public void writeToFile(String str) {

        }

        public String getCurrentSoftwareVersion() {
            return RobobuggyConfigFile.ALICE_LIBRARY_VERSION;
        }
    }


	/**
	 * Called to translate a peeled message to a JSON object
	 * @param message {@link String} of the peeled message
	 * @return {@link JSONObject} representing the string
	 */
	@SuppressWarnings("unchecked")
	public static JSONObject translatePeelMessageToJObject(String message) {
		// TODO Auto-generated method stub
		JSONObject data = new JSONObject();
		JSONObject params = new JSONObject();
		if (message.contains(GuiLoggingButtonMessage.LoggingMessage.START.toString())) {
			params.put("logging_status", "start");
		}
		else {
			params.put("logging_status", "stop");
		}
		data.put("timestamp", message.split(",")[1]);
		data.put("params", params);
		return data;
	}

}
