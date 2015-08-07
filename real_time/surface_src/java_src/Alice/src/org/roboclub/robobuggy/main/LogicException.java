package org.roboclub.robobuggy.main;

/**
 * 
 * @author Trevor Decker
 * @version 0.0
 *
 * This Class implments our own version of an exception so that we can tell the system 
 * that a logic error occured. Convenitly we will only need to change the action for that
 *  error in one place. So for example we can handle displaying the appropriate error to 
 *  the user, possibly logging the error, or possible causing a warning light to turn on.  
 */
public class LogicException extends Exception {
public LogicException(String error,MESSAGE_LEVEL level){
	System.out.println(error);
	//only halt the program if it is an exception 
	if(level == MESSAGE_LEVEL.exception){
		assert(false);
	}
}
}
