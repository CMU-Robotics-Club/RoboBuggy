package com.roboclub.robobuggy.simulation;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.io.UnsupportedEncodingException;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.BaseMessage;
import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.FingerPrintMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
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
import com.roboclub.robobuggy.ui.Gui;

/**
 * Class used for playing back old log files. It does this by reading BuggyROS
 * messages stored in the log file and reinjecting them into the BuggyROS network.
 */
public class SensorPlayer extends Thread {

    private String path;
    private Gson translator;
    private double playbackSpeed;

    private Publisher imuPub;
    private Publisher magPub;
    private Publisher gpsPub;
    private Publisher encoderPub;
    private Publisher brakePub;
    private Publisher steeringPub;
    private Publisher loggingButtonPub;
    private Publisher logicNotificationPub;

    // ---- Log File Defaults ----
    private static final String TERMINATING_VERSION_ID = "STOP";


    /**
     * Construct a new {@link SensorPlayer} object
     * @param filePath {@link String} of the name and location of the log file
     * @param playbackSpeed the initial playback speed
     */
    public SensorPlayer(String filePath, int playbackSpeed) {

        imuPub = new Publisher(NodeChannel.IMU.getMsgPath());
        magPub = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());
        gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
        encoderPub = new Publisher(NodeChannel.ENCODER.getMsgPath());
        brakePub = new Publisher(NodeChannel.BRAKE.getMsgPath());
        steeringPub = new Publisher(NodeChannel.DRIVE_CTRL.getMsgPath());
        loggingButtonPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
        logicNotificationPub = new Publisher(NodeChannel.LOGIC_NOTIFICATION.getMsgPath());

        translator = new GsonBuilder().create();

        new RobobuggyLogicNotification("initializing the SensorPlayer", RobobuggyMessageLevel.NOTE);
        path = filePath;
        File f = new File(path);
        if(!f.exists()) {
            new RobobuggyLogicNotification("File doesn't exist!", RobobuggyMessageLevel.EXCEPTION);
        }

        setupPlaybackTrigger();

        this.playbackSpeed = playbackSpeed;
    }


    /**
     * Sets up the playback trigger - hitting the start button will go through 1 iteration of a log file
     */
    public void setupPlaybackTrigger() {
        new Subscriber(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                GuiLoggingButtonMessage message = (GuiLoggingButtonMessage)m;
                if (message.getLoggingMessage().equals(GuiLoggingButtonMessage.LoggingMessage.START)) {
                    if (!SensorPlayer.this.isAlive()) {
                        SensorPlayer.this.start();
                    }
                }
            }
        });
    }

    @Override
    public void run() {

        try {

            InputStreamReader fileReader = new InputStreamReader(new FileInputStream(new File(path)), "UTF-8");
            JsonObject logFile = translator.fromJson(fileReader, JsonObject.class);

            if(!PlayBackUtil.validateLogFileMetadata(logFile)) {
                new RobobuggyLogicNotification("Log file doesn't have the proper header metadata!", RobobuggyMessageLevel.EXCEPTION);
                return;
            }

            long prevSensorTime = -1;

            JsonArray sensorDataArray = logFile.getAsJsonArray("sensor_data");
            for (JsonElement sensorAsJElement: sensorDataArray) {
                if(sensorAsJElement.isJsonObject()){
                    JsonObject sensorDataJson = sensorAsJElement.getAsJsonObject();
                    String versionID = sensorDataJson.get("VERSION_ID").getAsString();

                    Message transmitMessage = null;

                    switch (versionID) {
                        case BrakeControlMessage.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, BrakeControlMessage.class);
                            break;
                        case BrakeMessage.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, BrakeMessage.class);
                            break;
                        case MagneticMeasurement.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, MagneticMeasurement.class);
                        case DriveControlMessage.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, DriveControlMessage.class);
                            break;
                        case EncoderMeasurement.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, EncoderMeasurement.class);
                            break;
                        case FingerPrintMessage.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, FingerPrintMessage.class);
                            break;
                        case GpsMeasurement.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, GpsMeasurement.class);
                            break;
                        case GuiLoggingButtonMessage.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, GuiLoggingButtonMessage.class);
                            break;
                        case ImuMeasurement.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, ImuMeasurement.class);
                            break;
                        case GPSPoseMessage.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, GPSPoseMessage.class);
                            break;
                        case RemoteWheelAngleRequest.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, RemoteWheelAngleRequest.class);
                            break;
                        case ResetMessage.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, ResetMessage.class);
                            break;
                        case RobobuggyLogicNotificationMeasurement.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, RobobuggyLogicNotificationMeasurement.class);
                            break;
                        case StateMessage.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, StateMessage.class);
                            break;
                        case SteeringMeasurement.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, SteeringMeasurement.class);
                            break;
                        case WheelAngleCommandMeasurement.VERSION_ID:
                            transmitMessage = translator.fromJson(sensorDataJson, WheelAngleCommandMeasurement.class);
                            break;
                        case TERMINATING_VERSION_ID:
                            new RobobuggyLogicNotification("Stopping playback, hit a STOP", RobobuggyMessageLevel.NOTE);
                            break;
                        default:
                            transmitMessage = new BaseMessage() {
                                @Override
                                public String toLogString() {
                                    return null;
                                }

                                @Override
                                public Message fromLogString(String str) {
                                    return null;
                                }
                            };
                            new RobobuggyLogicNotification("Couldn't parse a sensor found in the file: " + versionID, RobobuggyMessageLevel.WARNING);
                            break;
                    }

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
                        long timeDiff = currentSensorTime - prevSensorTime;
                        if (timeDiff > 10) {
                            Thread.sleep((long) (timeDiff / playbackSpeed));
                        }
                        prevSensorTime = currentSensorTime;
                    }


                    switch (versionID) {
                        case BrakeMessage.VERSION_ID:
                            brakePub.publish(transmitMessage);
                            break;
                        case EncoderMeasurement.VERSION_ID:
                            encoderPub.publish(transmitMessage);
                            break;
                        case GpsMeasurement.VERSION_ID:
                            gpsPub.publish(transmitMessage);
                            break;
                        case GuiLoggingButtonMessage.VERSION_ID:
                            loggingButtonPub.publish(transmitMessage);
                            break;
                        case ImuMeasurement.VERSION_ID:
                            imuPub.publish(transmitMessage);
                            break;
                        case RobobuggyLogicNotificationMeasurement.VERSION_ID:
                            logicNotificationPub.publish(transmitMessage);
                            break;
                        case SteeringMeasurement.VERSION_ID:
                            steeringPub.publish(transmitMessage);
                            break;
                        default:
                            break;
                    }

                }
            }

        } catch (FileNotFoundException e) {
            new RobobuggyLogicNotification("SensorPlayer couldn't find log file!", RobobuggyMessageLevel.EXCEPTION);
        } catch (InterruptedException e) {
            new RobobuggyLogicNotification("SensorPlayer was interrupted", RobobuggyMessageLevel.WARNING);
        } catch (UnsupportedEncodingException e) {
            new RobobuggyLogicNotification("Log file had unsupported encoding", RobobuggyMessageLevel.EXCEPTION);
        }
    }



    /**
     * gets the new playback speed from the GUI and puts it into playbackSpeed
     */
    public void getNewPlaybackSpeed() {
        playbackSpeed = Gui.getInstance().getMainGuiWindow().getCtrlPanel().getLoggingPanel().getPlaybackSpeed();
    }
}
