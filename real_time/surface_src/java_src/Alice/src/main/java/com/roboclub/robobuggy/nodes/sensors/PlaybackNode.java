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

    /**
     * Constructor for sensor player
     *
     * @param filePath      the file to play
     * @param playBackSpeed the playback speed
     */
    public PlaybackNode(String filePath, double playBackSpeed) {
        super(new BuggyBaseNode(NodeChannel.PLAYBACK), "Playback");
        //open up the log file

        new RobobuggyLogicNotification("initializing the SensorPlayer", RobobuggyMessageLevel.NOTE);
        Thread thread = new Thread() {
            public void run() {
                Gson translator = new GsonBuilder().create();//TODO don't create a new one for every call of this function
                // open input stream test.txt for reading purpose.
                try {
                    InputStream is = new FileInputStream(filePath);
                    // create new input stream reader
                    InputStreamReader isr = new InputStreamReader(is, "UTF-8");
                    //the file was created so lets try and read it
                    BufferedReader br = new BufferedReader(isr);
                    String nextLine;
                    nextLine = br.readLine();  //reads {
                    if (nextLine != null)
                        nextLine = br.readLine();  // name
                    else {
                        new RobobuggyLogicNotification("File wasn't a log file!", RobobuggyMessageLevel.EXCEPTION);
                        br.close();
                        return;
                    }
                    if (nextLine != null)
                        nextLine = br.readLine();  //schema version
                    if (nextLine != null)
                        nextLine = br.readLine();  //date recorded
                    if (nextLine != null)
                        nextLine = br.readLine();  //software version
                    if (nextLine != null)
                        nextLine = br.readLine();  //Sensor_data
                    if (nextLine != null)
                        nextLine = br.readLine();
                    if (nextLine != null)
                        nextLine = nextLine.substring(0, nextLine.length() - 1);
                    JsonObject sensorObject = translator.fromJson(nextLine, JsonObject.class);
                    long startTimeSensor = sensorObject.get("timestamp").getAsLong();

                    long startTimeReal = new Date().getTime();
                    nextLine = br.readLine();
                    while (nextLine != null && !nextLine.startsWith("]")) {
                        try {
                            //has not reached the end of the log yet
                            //removes the comma at the end of the line
                            nextLine = nextLine.substring(0, nextLine.length() - 1);
                            long now = new Date().getTime();
                            sensorObject = translator.fromJson(nextLine, JsonObject.class);
                            long dt = now - startTimeReal;
                            PlayBackUtil.parseSensorLog(sensorObject, translator, dt, startTimeSensor,
                                    RobobuggyConfigFile.getPlayBackSpeed());
                            //TODO deal with delay stuff

                            nextLine = br.readLine();
                        } catch (IOException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                        } catch (InterruptedException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                        }
                    }//while
                    br.close();


                } catch (FileNotFoundException e) {
                    new RobobuggyLogicNotification("File doesn't exist!", RobobuggyMessageLevel.EXCEPTION);
                    e.printStackTrace();
                } catch (IOException e1) {
                    // TODO Auto-generated catch block
                    e1.printStackTrace();
                }
            }//run

        };//thread

        thread.start();


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
