package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.messages.LogicExceptionMeasurment;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;

/**
 * 
 * @author Trevor Decker
 * @version 0.0
 *
 * This class implements our own version of an exception so that we can tell the system 
 * that a logic error occurred. conveniently we will only need to change the action for that
 *  error in one place. So for example we can handle displaying the appropriate error to 
 *  the user, possibly logging the error, or possible causing a warning light to turn on.  
 */
public class LogicException extends Exception {
	private static Publisher errorPub;
	
	//must be run before LogicException can be called 
public static void setupLogicException(SensorChannel sensor){
	errorPub = new Publisher(sensor.getMsgPath());
}
	
public LogicException(String error,MESSAGE_LEVEL level){
	System.out.println(error);
	errorPub.publish(new LogicExceptionMeasurment(error, level));
	
	//only halt the program if it is an exception 
	if(level == MESSAGE_LEVEL.exception){
		assert(false);
	}
}
}
