package com.roboclub.robobuggy.actuators;

import com.roboclub.robobuggy.main.config;

/**
 * 
 * @author Trevor Decker
 * 
 * @version 0.5
 * 
 * CHANGELONG:
 * 
 * DESCRIPTION:
 * Class for keeping track of the status of the brake and providing an api for actions that use the brake
 * Has access to Arduino which controls the solenoid for the brake.  
 * 
 */

public class brake extends actuator{
	//assumes the system is at full pressure when brake is created
	private boolean state;//true if down, false if up 
	private boolean desiredState = false;//true if down, false if up 
	private byte timesAllowedToDeploy = config.BRAKES_PER_FULL_PRESSURE; 
	
	/*** constructor for brake class, registers a connection with the brake 
	     controlling arduino will raise a fatal error if connection is not achieved. **/
	public brake(){
		super();
		//TODO create connection with arduino that controls brakes 
		//TODO raise fatal error if connection to arduino is not achieved 
		//TODO set state based on actual status of the break
	}
	
	/*** the system is only allowed to brake when we believe that the system has 
	     Enough pressure to stop the buggy 
	     @return true if it possible to brake, false otherwise 
	     */
	public boolean canWeBrake(){
		if(timesAllowedToDeploy > 0){
			return true;
		}else{
			return false;
		}
	}
	
	/*** the system attempts to brake, if this fails then the a fatal error will be raised 
	 *   @throws exception if we can not deploy the brakes */	
	public void deployBrake() throws Exception{
		desiredState = true;
		if(!canWeBrake()){
			throw new Exception("trying to deploy breaks when we should be out of preasure or unable to Break");
		}
		timesAllowedToDeploy--;
		//TODO send command to break 
	}
	
	/*** the system attempts to brake, if this fails then a fatal error will be raised */
	public void releaseBrake(){
		desiredState = false;
		//TODO implment 
	}
	

	
}
