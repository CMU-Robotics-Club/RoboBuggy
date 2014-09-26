package com.roboclub.robobuggy.pathPlanning;

public interface PathPlanner {
	public void PathPlanner(); //init the path planner setting up all resources 
	public void getNextDesiredLocation();
	public void setCurrentState();
	public void calculatePlan(long time);//does processing for time miliseconds
	public void getStatus();  //return structure including any status messages
	
}
