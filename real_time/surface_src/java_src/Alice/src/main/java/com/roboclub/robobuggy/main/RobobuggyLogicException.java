package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.messages.RobobuggyLogicExceptionMeasurment;
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
 *  
 *   Note that we do NOT throw this, as creating a new instance will publish to the error channel
 */
public class RobobuggyLogicException {
	private static Publisher errorPub;
	
	//must be run before LogicException can be called 
public static void setupLogicException(SensorChannel sensor){
	errorPub = new Publisher(sensor.getMsgPath());
}
	
public RobobuggyLogicException(String error,MessageLevel level){
	System.out.println(error);
	errorPub.publish(new RobobuggyLogicExceptionMeasurment(error, level));
	
	//only halt the program if it is an exception 
	if(level == MessageLevel.EXCEPTION){
		assert(false);
	}
}
}
