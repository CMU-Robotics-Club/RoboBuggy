package com.roboclub.robobuggy.actuators;

/**
 * 
 * @author Trevor Decker
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: abstract Class for keeping track of the status of an
 *          actuator and providing an api for actions that use that actuator.
 *          This class has access to Arduino which controls the actuator.
 */

abstract class actuator {
	/**
	 * represents if the system is in an error or not, true for in error, false
	 * for no error
	 */
	private boolean errorState;

	/***
	 * An error caused relating to this actuator this causes a fatal error
	 * 
	 * @param description
	 *            of error to throw
	 * @throws Exception
	 */
	public void notifyBrakeError(String error) throws Exception {
		setErrorState(true);
		throw new Exception(error);
	}

	/*** constructor for a new actuator, assumes that system is not in error */
	actuator() {
		setErrorState(false);
	}

	/*** updates the systems error status */
	private void setErrorState(boolean newErrorState) {
		errorState = newErrorState;
	}

	/***
	 * evaluates to the systems error status, true for in error, false for no
	 * error
	 */
	public boolean getErrorState() {
		return errorState;
	}
}
