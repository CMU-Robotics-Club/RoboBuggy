package com.roboclub.robobuggy.nodes.sensors;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.simulation.PlayBackUtil;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Date;

/**
 * a tool for reading log files line by line
 */
public class PlaybackNode extends BuggyDecoratorNode {

    private Thread playbackThread;

    /**
     * Constructor for sensor player
     *  @param filePath      the file to play
     *
     */
    public PlaybackNode(String filePath) {
        super(new BuggyBaseNode(NodeChannel.PLAYBACK), "Playback");
        //open up the log file
        Gson translator = new GsonBuilder().create();

        new RobobuggyLogicNotification("Initializing the PlaybackNode", RobobuggyMessageLevel.NOTE);
        playbackThread = new Thread() {
            public void run() {
                // open input stream test.txt for reading purpose.
                BufferedReader br;
                try {
                    InputStream is = new FileInputStream(filePath);
                    // create new input stream reader
                    InputStreamReader isr = new InputStreamReader(is, "UTF-8");
                    //the file was created so lets try and read it
                    br = new BufferedReader(isr);
                    String nextLine;

                    // read the metadata line
                    nextLine = br.readLine();
                    if (nextLine == null) {
                        new RobobuggyLogicNotification("Empty log file!", RobobuggyMessageLevel.EXCEPTION);
                        br.close();
                        return;
                    }
                    JsonObject metadataObject = translator.fromJson(nextLine, JsonObject.class);
                    String message =
                            "name : " + metadataObject.get("name").getAsString() +
                            " schema : " + metadataObject.get("schema_version").getAsString() +
                            " date_recorded " + metadataObject.get("date_recorded").getAsString() +
                            " git hash " + metadataObject.get("software_version").getAsString();
                    new RobobuggyLogicNotification("parsing log file:\n " + message, RobobuggyMessageLevel.NOTE);

                    long startTimeReal = new Date().getTime();
                    long startTimeSensor = -1;


                    // read the rest of the file
                    while ((nextLine = br.readLine()) != null) {
                        long now = new Date().getTime();
                        JsonObject sensorObject = translator.fromJson(nextLine, JsonObject.class);
                        long dt = now - startTimeReal;

                        if (sensorObject.get("VERSION_ID").getAsString().equals(LoggingNode.DATA_BREAKDOWN_ID) ||
                                sensorObject.get("VERSION_ID").getAsString().equals("STOP")) {
                            break;
                        }

                        if (startTimeSensor == -1) {
                            // get the first sensor reading, and get it's timestamp
                            startTimeSensor = sensorObject.get("timestamp").getAsLong();
                        }

                        try {
                            PlayBackUtil.parseSensorLog(sensorObject, translator, dt, startTimeSensor,
                                    RobobuggyConfigFile.getPlayBackSpeed());
                        } catch (InterruptedException e) {
                            new RobobuggyLogicNotification("Couldn't parse sensor object: " + nextLine, RobobuggyMessageLevel.WARNING);
                            e.printStackTrace();
                        }
                    }

                    // TODO do something clever with the data breakdown
                    br.close();


                } catch (FileNotFoundException e) {
                    new RobobuggyLogicNotification("File doesn't exist!", RobobuggyMessageLevel.EXCEPTION);
                    e.printStackTrace();
                } catch (IOException e1) {
                    new RobobuggyLogicNotification("Error while reading the file", RobobuggyMessageLevel.EXCEPTION);
                    e1.printStackTrace();
                }
            }

        };
    }

    /**
     * starts playing back the file
     */
    public void startPlayback() {
        playbackThread.start();
    }

    @Override
    protected boolean startDecoratorNode() {
        return false;
    }

    @Override
    protected boolean shutdownDecoratorNode() {
        return false;
    }
}
