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
import com.roboclub.robobuggy.messages.RemoteWheelAngleRequest;
import com.roboclub.robobuggy.messages.ResetMessage;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.messages.WheelAngleCommandMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * Class used for playing back old log files. It does this by reading BuggyROS
 * messages stored in the log file and reinjecting them into the BuggyROS network.
 */
public class SensorPlayer implements Runnable {

	private String path;
    private Gson translator;

	private Publisher imuPub;
	private Publisher gpsPub;
	private Publisher encoderPub;
	private Publisher brakePub;
	private Publisher steeringPub;
	private Publisher loggingButtonPub;
    private Publisher logicNotificationPub;

    // ---- Log File Defaults ----
    private static final String METADATA_NAME = "Robobuggy Data Logs";
    private static final String METADATA_SCHEMA_VERSION = "1.1";
    private static final String METADATA_HIGHLEVEL_SW_VERSION = "1.0.0";
    private static final String TERMINATING_VERSION_ID = "STOP";


	/**
	 * Construct a new {@link SensorPlayer} object
	 * @param filePath {@link String} of the name and location of the log file
	 */
	public SensorPlayer(String filePath) {

		imuPub = new Publisher(NodeChannel.IMU.getMsgPath());
		gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
		encoderPub = new Publisher(NodeChannel.ENCODER.getMsgPath());
		brakePub = new Publisher(NodeChannel.BRAKE.getMsgPath());
		steeringPub = new Publisher(NodeChannel.STEERING.getMsgPath());
		loggingButtonPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
        logicNotificationPub = new Publisher(NodeChannel.LOGIC_NOTIFICATION.getMsgPath());

        translator = new GsonBuilder().create();

		new RobobuggyLogicNotification("initializing the SensorPlayer", RobobuggyMessageLevel.NOTE);

		path = filePath;
		File f = new File(path);
		if(!f.exists()) {
			new RobobuggyLogicNotification("File doesn't exist!", RobobuggyMessageLevel.EXCEPTION);
		}
	}


	@Override
	public void run() {

        try {

            InputStreamReader fileReader = new InputStreamReader(new FileInputStream(new File(path)), "UTF-8");
            JsonObject logFile = translator.fromJson(fileReader, JsonObject.class);

            if(!validateLogFileMetadata(logFile)) {
                new RobobuggyLogicNotification("Log file doesn't have the proper header metadata!", RobobuggyMessageLevel.EXCEPTION);
                return;
            }

            long currentSensorTimeInMillis = -1;
            long sensorStartTimeInMilis = 0;

            JsonArray sensorDataArray = logFile.getAsJsonArray("sensor_data");
            for (JsonElement sensorAsJElement: sensorDataArray) {

                JsonObject sensorDataJson = sensorAsJElement.getAsJsonObject();
                String versionID = sensorDataJson.get("VERSION_ID").getAsString();

                Message transmitMessage = null;

                switch (versionID) {
                    case BrakeControlMessage.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, BrakeControlMessage.class);
                        break;
                    case BrakeMessage.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, BrakeMessage.class);
                        brakePub.publish(transmitMessage);
                        break;
                    case DriveControlMessage.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, DriveControlMessage.class);
                        break;
                    case EncoderMeasurement.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, EncoderMeasurement.class);
                        encoderPub.publish(transmitMessage);
                        break;
                    case FingerPrintMessage.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, FingerPrintMessage.class);
//                        encoderPub.publish(transmitMessage);
                        break;
                    case GpsMeasurement.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, GpsMeasurement.class);
                        gpsPub.publish(transmitMessage);
                        break;
                    case GuiLoggingButtonMessage.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, GuiLoggingButtonMessage.class);
                        loggingButtonPub.publish(transmitMessage);
                        break;
                    case ImuMeasurement.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, ImuMeasurement.class);
                        imuPub.publish(transmitMessage);
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
                        logicNotificationPub.publish(transmitMessage);
                        break;
                    case StateMessage.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, StateMessage.class);
                        break;
                    case SteeringMeasurement.VERSION_ID:
                        transmitMessage = translator.fromJson(sensorDataJson, SteeringMeasurement.class);
                        steeringPub.publish(transmitMessage);
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

                BaseMessage timeMessage = (BaseMessage) transmitMessage;

                if (currentSensorTimeInMillis != -1) {
                    if (timeMessage != null) {
                        currentSensorTimeInMillis = timeMessage.getTimestamp().getTime();
                        long sensorTimeFromPrev = currentSensorTimeInMillis - sensorStartTimeInMilis;

                        if(sensorTimeFromPrev > 0){
                            Thread.sleep(sensorTimeFromPrev);
                        }
                    }

                    sensorStartTimeInMilis = currentSensorTimeInMillis;
                }
                else {
                    assert timeMessage != null;
                    currentSensorTimeInMillis = timeMessage.getTimestamp().getTime();
                    sensorStartTimeInMilis = currentSensorTimeInMillis;
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

    private boolean validateLogFileMetadata(JsonObject logFile) {

        if (!logFile.get("name").getAsString().equals(METADATA_NAME)) {
            return false;
        }
        if (!logFile.get("schema_version").getAsString().equals(METADATA_SCHEMA_VERSION)) {
            return false;
        }
        if (!logFile.get("software_version").getAsString().equals(METADATA_HIGHLEVEL_SW_VERSION)) {
            return false;
        }

        return true;
    }
}
