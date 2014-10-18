package com.roboclub.robobuggy.actuators;

import com.roboclub.robobuggy.main.config;

public class brake {
	//assumes the system is at full pressure when brake is created
	private boolean state;//true if down, false if up 
	private boolean desiredState = false;//true if down, false if up 
	private byte timesAllowedToDeploy = config.BRAKES_PER_FULL_PRESSURE; 
	
	
	public brake(){
		//TODO 
		//TODO set state based on acutal status of the break
	}
	
	//the system is only allowed to brake when we belive that the system has 
	// enougth preassure to stop the buggy 
	public boolean canWeBrake(){
		if(timesAllowedToDeploy > 0){
			return true;
		}else{
			return false;
		}
	}
	
	public void deployBrake() throws Exception{
		desiredState = true;
		if(!canWeBrake()){
			throw new Exception("trying to deploy breaks when we should be out of preasure or unable to Break");
		}
		timesAllowedToDeploy--;
		//TODO send command to break 
	}
	
	public void releaseBrake(){
		desiredState = false;
	}
	
	//TODO if the system breaks due other then us telling it to we should detect that,
	// and use it set the break state
	
	
}
