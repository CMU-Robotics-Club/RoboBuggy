package com.roboclub.robobuggy.simulation;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.BaseMessage;
import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.FingerPrintMessage;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.messages.RemoteWheelAngleRequest;
import com.roboclub.robobuggy.messages.ResetMessage;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.messages.WheelAngleCommandMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.UnsupportedEncodingException;

@Deprecated
/**
 * Class used for playing back old log files. It does this by reading BuggyROS
 * messages stored in the log file and reinjecting them into the BuggyROS network.
 */
public class SensorPlayer extends Thread {

    private String path;
    private double playbackSpeed;
    private boolean isPaused = false;



    JsonArray sensorDataArray;

    // ---- Log File Defaults ----
    private static final String TERMINATING_VERSION_ID = "STOP";


    /**
     * Construct a new {@link SensorPlayer} object
     * @param filePath {@link String} of the name and location of the log file
     * @param playbackSpeed the initial playback speed
     */
    public SensorPlayer(String filePath, int playbackSpeed) {


        new RobobuggyLogicNotification("initializing the SensorPlayer", RobobuggyMessageLevel.NOTE);
        path = filePath;
        File f = new File(path);
        if(!f.exists()) {
            new RobobuggyLogicNotification("File doesn't exist!", RobobuggyMessageLevel.EXCEPTION);
        }

        setupPlaybackTrigger();

        JsonObject logFile  = null;
        try {
            logFile = Util.readJSONFile(path);
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        if(!PlayBackUtil.validateLogFileMetadata(logFile)) {
            new RobobuggyLogicNotification("Log file doesn't have the proper header metadata!", RobobuggyMessageLevel.EXCEPTION);
            return;
        }


        sensorDataArray = logFile.getAsJsonArray("sensor_data");

        this.playbackSpeed = playbackSpeed;
    }


    /**
     * Sets up the playback trigger - hitting the start button will go through 1 iteration of a log file
     */
    public void setupPlaybackTrigger() {
        new Subscriber("playback", NodeChannel.GUI_LOGGING_BUTTON.getMsgPath(), new MessageListener() {
            @Override
            public synchronized void actionPerformed(String topicName, Message m) {
                GuiLoggingButtonMessage message = (GuiLoggingButtonMessage)m;
                if (message.getLoggingMessage().equals(GuiLoggingButtonMessage.LoggingMessage.START)) {

                    //unpause it if we paused, or start fresh if we haven't paused
                    if (SensorPlayer.this.isPaused) {
                        SensorPlayer.this.isPaused = false;
                        new RobobuggyLogicNotification("Resumed playback", RobobuggyMessageLevel.NOTE);
                    }
                    else {
                        SensorPlayer.this.start();
                        new RobobuggyLogicNotification("Started playback", RobobuggyMessageLevel.NOTE);
                    }
                }
                else if (message.getLoggingMessage().equals(GuiLoggingButtonMessage.LoggingMessage.STOP)) {
                    if (!SensorPlayer.this.isPaused) {
                        SensorPlayer.this.isPaused = true;
                        new RobobuggyLogicNotification("Paused playback", RobobuggyMessageLevel.NOTE);
                    }
                    else {
                        new RobobuggyLogicNotification("Quit playback, please restart GUI to reset", RobobuggyMessageLevel.NOTE);
                    }
                }

            }
        });
    }

    @Override
    public void run() {

        Gson translator = new GsonBuilder().create();
        /*
       try {

            long prevSensorTime = -1;

            for (JsonElement sensorAsJElement: sensorDataArray) {
                
                // spin in a tight loop until we've unpaused
                while (isPaused) {
                    Thread.sleep(100);
                }

                if(sensorAsJElement.isJsonObject()){
                	Message transmitMessage =  PlayBackUtil.parseSensorLog(sensorAsJElement.getAsJsonObject(),translator, );
                    getNewPlaybackSpeed();
                    BaseMessage timeMessage = (BaseMessage) transmitMessage;

                    if (timeMessage == null) {
                        continue;
                    }

                    if (prevSensorTime == -1) {
                        prevSensorTime = timeMessage.getTimestamp().getTime();
                    }
                    else {
                        long currentSensorTime = timeMessage.getTimestamp().getTime();
                        long timeDiff = currentSensorTime - prevSensorTime;  //in milliseconds
                        long dt = (long) (timeDiff/playbackSpeed);
                        if (dt > 100) {
                            Thread.sleep(dt);
                            prevSensorTime = currentSensorTime;
                        }
                         
                    }


                   

                }
            }

//        } catch (FileNotFoundException e) {
//            new RobobuggyLogicNotification("SensorPlayer couldn't find log file!", RobobuggyMessageLevel.EXCEPTION);
       } catch (InterruptedException e) {
            new RobobuggyLogicNotification("SensorPlayer was interrupted", RobobuggyMessageLevel.WARNING);
//        } catch (UnsupportedEncodingException e) {
//            new RobobuggyLogicNotification("Log file had unsupported encoding", RobobuggyMessageLevel.EXCEPTION);
        }
        */
    }


    /**
     * gets the new playback speed from the GUI and puts it into playbackSpeed
     */
    public void getNewPlaybackSpeed() {
    	playbackSpeed = RobobuggyConfigFile.getPlayBackSpeed();
    }
}
