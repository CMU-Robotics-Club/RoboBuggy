package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

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
public class RobobuggyLogicNotification {
	private static Publisher errorPub;
	
	//must be run before LogicException can be called 
	/**
	 * Sets up the {@link RobobuggyLogicNotification}. Must be called before a new
	 * {@link RobobuggyLogicNotification} can be constructed.
	 */
	private static void setupLogicException(){
		errorPub = new Publisher(NodeChannel.LOGIC_NOTIFICATION.getMsgPath());
	}
	
	/**
	 * Constructs a new {@link RobobuggyLogicNotification}
	 * @param message description of the message that occurred
	 * @param level {@link RobobuggyMessageLevel} of the
	 *  {@link RobobuggyLogicNotification}
	 */
	public RobobuggyLogicNotification(String message, RobobuggyMessageLevel level){
		if(shouldMessageBeDisplayed(level)){
			//displays the error message to the java console 
			System.out.println(message);
		}

		if(errorPub ==null){
			setupLogicException();
			}
		//the message is always published 
		errorPub.publish(new RobobuggyLogicNotificationMeasurement(message, level));

		/*TODO: do something logical this segment was causing an infinite loop where the robot was being shut down and then restarted
		//only halt the program if it is an message
		if(level == RobobuggyMessageLevel.EXCEPTION){
		//	Robot.getInstance().shutDown();  
		}
		*/
	}

	//a function to check if a message level is significant enough to display to the user
	//note that this function will need to be updated if message level ever 
	private boolean shouldMessageBeDisplayed(RobobuggyMessageLevel level){
		switch(level){
		case EXCEPTION:
			switch(RobobuggyConfigFile.REPORTING_LEVEL){
			case EXCEPTION:
				return true;
			case WARNING:
				return false;
			case NOTE:
				return false;
			default:
				return true;
			}
		case WARNING:
			switch(RobobuggyConfigFile.REPORTING_LEVEL){
			case EXCEPTION:
				return true;
			case WARNING:
				return true;
			case NOTE:
				return false;
			default:
				return true;
			}
		case NOTE:
			switch (RobobuggyConfigFile.REPORTING_LEVEL) {
				case EXCEPTION:
					return true;
				case WARNING:
					return true;
				case NOTE:
					return true;		
				default:
					return true;
				}
        default:
            return true;
        }
	}


}
