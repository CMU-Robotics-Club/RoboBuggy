package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.messages.RobobuggyLogicExceptionMeasurment;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.sun.org.apache.bcel.internal.generic.RETURN;

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
public static void setupLogicException(NodeChannel sensor){
	errorPub = new Publisher(sensor.getMsgPath());
}
	
public RobobuggyLogicException(String error,MessageLevel level){
	if(shouldMessageBeDisplayed(level)){
		//displays the error message to the jave console 
		System.out.println(error);
	}
	//the message is always published 
	errorPub.publish(new RobobuggyLogicExceptionMeasurment(error, level));
	
	//only halt the program if it is an exception 
	if(level == MessageLevel.EXCEPTION){
		assert(false); //todo shutdown in a more graceful way
	}
}

//a function to check if a message level is signfigent enough to display to the user
//note that this function will need to be updated if message level ever 
private boolean shouldMessageBeDisplayed(MessageLevel level){
	switch(level){
	case EXCEPTION:
		switch(config.REPORTING_LEVEL){
		case EXCEPTION:
			return true;
		case WARNING:
			return false;
		case NOTE:
			return false;
		}
		break;
	case WARNING:
		switch(config.REPORTING_LEVEL){
		case EXCEPTION:
			return true;
		case WARNING:
			return true;
		case NOTE:
			return false;
		}
		break;
	case NOTE:
		switch (config.REPORTING_LEVEL) {
		case EXCEPTION:
			return true;
		case WARNING:
			return true;
		case NOTE:
			return true;
		}		
			break;
	}
	return true; 
}

}
