package com.roboclub.robobuggy.actuators;

/**
 * 
 * @author Trevor Decker
 * 
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: 
 * Class for a light that the surface can control this class tracks the status of the light and provides an api
 * for actions that use the brake the light.  THis class has access to Arduino which controls the light.  
 * 
 */


public class light extends actuator{
	/** true for light on, false for light off */
	boolean state;
	
	/**
	 * creates the light object, as input takes what state the light should be set to.
	 * true for on false for off
	 * @param lightState 
	 */
	public light(boolean lightState){
		super();
		state = lightState;
		//TODO setup arduino com
	}
	
	/**
	 * returns the light state true for on false for off
	 * @return the state of the light 
	 */
	public boolean getLightState(){
		return state;
	}
	
	/**
	 * sets the light state true for on false for off 
	 * @param newLightState
	 */
	public void setLightState(boolean newLightState){
		state = newLightState;
	}
	
}
