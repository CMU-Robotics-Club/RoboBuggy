package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.messages.RobobuggyLogicExceptionMeasurment;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;

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
	/**
	 * Sets up the {@link RobobuggyLogicException}. Must be called before a new
	 * {@link RobobuggyLogicException} can be constructed.
	 * @param channel {@link NodeChannel} used to log the
	 * {@link RobobuggyLogicException}s on
	 */
	public static void setupLogicException(NodeChannel channel){
		errorPub = new Publisher(channel.getMsgPath());
	}
	
	/**
	 * Constructs a new {@link RobobuggyLogicException}
	 * @param exception description of the exception that occurred
	 * @param level {@link RobobuggyMessageLevel} of the
	 *  {@link RobobuggyLogicException}
	 */
	public RobobuggyLogicException(String exception, RobobuggyMessageLevel level){
		if(shouldMessageBeDisplayed(level)){
			//displays the error message to the java console 
			System.out.println(exception);
		}
		//the message is always published 
		errorPub.publish(new RobobuggyLogicExceptionMeasurment(exception, level));
		
		//only halt the program if it is an exception 
		if(level == RobobuggyMessageLevel.EXCEPTION){
			assert(false); //todo shutdown in a more graceful way
		}
	}

	//a function to check if a message level is signfigent enough to display to the user
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
				System.out.println("Unknown MessageLevel. Printing");
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
				System.out.println("Unknown MessageLevel. Printing");
				return true;
			}
		case NOTE:
			switch (RobobuggyConfigFile.REPORTING_LEVEL) {
			case EXCEPTION:
				switch(RobobuggyConfigFile.REPORTING_LEVEL){
				case EXCEPTION:
					return true;
				case WARNING:
					return false;
				case NOTE:
					return false;
				default:
					System.out.println("Unknown MessageLevel. Printing");
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
					System.out.println("Unknown MessageLevel. Printing");
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
					System.out.println("Unknown MessageLevel. Printing");
					return true;
				}
			default:
				System.out.println("Unknown MessageLevel. Printing");
				return true;
			}
		default:
			System.out.println("Unknown MessageLevel. Printing");
			return true;
		}
	}
}
