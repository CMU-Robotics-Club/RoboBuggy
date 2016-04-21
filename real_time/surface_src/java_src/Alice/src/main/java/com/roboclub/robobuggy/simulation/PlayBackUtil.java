package com.roboclub.robobuggy.simulation;


import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.FingerPrintMessage;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.IMUAngularPositionMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.messages.RemoteWheelAngleRequest;
import com.roboclub.robobuggy.messages.ResetMessage;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.messages.WheelAngleCommandMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

/**
 * utilities for playback
 */
public class PlayBackUtil {
	  private static final String METADATA_NAME = "Robobuggy Data Logs";
	  private static final String METADATA_SCHEMA_VERSION = "1.1";
	  private static final String METADATA_HIGHLEVEL_SW_VERSION = "1.0.0";
	  private static final  Publisher IMU_PUB = new Publisher(NodeChannel.IMU.getMsgPath());
	  private static final Publisher  MAG_PUB = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());;
	  private static final Publisher  GPS_PUB = new Publisher(NodeChannel.GPS.getMsgPath());;
	  private static final  Publisher ENCODER_PUB  = new Publisher(NodeChannel.ENCODER.getMsgPath());
	  private static final Publisher  BRAKE_PUB = new Publisher(NodeChannel.BRAKE_STATE.getMsgPath());
	  private static final Publisher  STEERING_PUB = new Publisher(NodeChannel.STEERING.getMsgPath());
	  private static final Publisher  STEERING_COMMAND_PUB = new Publisher(NodeChannel.STEERING_COMMANDED.getMsgPath());
	  private static final Publisher  LOGGING_BUTTON_PUB  = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
	  private static final Publisher  LOGIC_NOTIFICIATION_PUB = new Publisher(NodeChannel.LOGIC_NOTIFICATION.getMsgPath());



    /**
     * validates the log file metadata
     * @param logFile the log file to validate
     * @return whether or not the log file is valid
     */
    public static boolean validateLogFileMetadata(JsonObject logFile) {
    	if(logFile == null){
    		return false;
    	}
    	
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
    

    /**
     * reads a sensor log and outputs the next message, if the next message is not suppose to appear for some time then this method will block until that time
     * @param sensorDataJson the json object of sensor data
     * @param translator translator object
     * @param playBacktime the time the playback should play until
     * @param sensorStartTime the time the sensor playback started at
     * @param playBackSpeed the speed to playback at
     * @return the message from the log
     * @throws InterruptedException timing didn't work
     */
    public static Message parseSensorLog(JsonObject sensorDataJson,Gson translator,long playBacktime,
    		long sensorStartTime,double playBackSpeed) throws InterruptedException{
            // wait until the time this message is supposed to be sent
			long sensorTime = sensorDataJson.get("timestamp").getAsLong();
            long sensorDt = (sensorTime-sensorStartTime);
            long dt = (long) (sensorDt/playBackSpeed) - playBacktime;
            if(dt> 10){ //Milliseconds
                Thread.sleep(dt);
            }

            // dispatch this message depending on version, maybe topic
            String versionID = sensorDataJson.get("VERSION_ID").getAsString();
            Message transmitMessage = null;
            switch (versionID) {
                case BrakeControlMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, BrakeControlMessage.class);
                    break;
                case BrakeMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, BrakeMessage.class);
                    BRAKE_PUB.publish(transmitMessage);
                    break;
                case MagneticMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, MagneticMeasurement.class);
                    MAG_PUB.publish(transmitMessage);
                    break;
                case DriveControlMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, DriveControlMessage.class);
                    break;
                case EncoderMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, EncoderMeasurement.class);
                    ENCODER_PUB.publish(transmitMessage);
                    break;
                case FingerPrintMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, FingerPrintMessage.class);
                    break;
                case GpsMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, GpsMeasurement.class);
                    GPS_PUB.publish(transmitMessage);
                    break;
                case GuiLoggingButtonMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, GuiLoggingButtonMessage.class);
                    LOGGING_BUTTON_PUB.publish(transmitMessage);
                    break;
                case ImuMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, ImuMeasurement.class);
                    IMU_PUB.publish(transmitMessage);
                    break;
                case GPSPoseMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, GPSPoseMessage.class);
                    break;
                case RemoteWheelAngleRequest.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, RemoteWheelAngleRequest.class);
                    break;
                case IMUAngularPositionMessage.VERSION_ID:
                	transmitMessage = translator.fromJson(sensorDataJson, IMUAngularPositionMessage.class);
                	break;
                case ResetMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, ResetMessage.class);
                    break;
                case RobobuggyLogicNotificationMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, RobobuggyLogicNotificationMeasurement.class);
                    LOGIC_NOTIFICIATION_PUB.publish(transmitMessage);
                    break;
                case StateMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, StateMessage.class);
                    break;
                case SteeringMeasurement.VERSION_ID:
                    // want to filter by steering feedback
                    if(sensorDataJson.get("topicName").getAsString().equals(NodeChannel.STEERING.getMsgPath())) {
                        transmitMessage = translator.fromJson(sensorDataJson, SteeringMeasurement.class);
                        STEERING_PUB.publish(transmitMessage);
                    }
                    else if(sensorDataJson.get("topicName").getAsString().equals(NodeChannel.STEERING_COMMANDED.getMsgPath())) {
                        transmitMessage = translator.fromJson(sensorDataJson, SteeringMeasurement.class);
                        STEERING_COMMAND_PUB.publish(transmitMessage);
                    }
                    // any other type of steering message we ignore
                    else {
                        return transmitMessage;
                    }
                    break;
                case WheelAngleCommandMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, WheelAngleCommandMeasurement.class);
                    break;
             /*   case TERMINATING_VERSION_ID:
                    new RobobuggyLogicNotification("Stopping playback, hit a STOP", RobobuggyMessageLevel.NOTE);
                    return;
                    */
                default:
                    break;
            }
			return transmitMessage;
    }
	
}
