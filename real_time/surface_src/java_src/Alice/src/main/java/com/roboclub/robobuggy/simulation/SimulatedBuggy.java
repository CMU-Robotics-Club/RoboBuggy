package com.roboclub.robobuggy.simulation;

import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.util.Date;
import java.util.TimerTask;

/**
 * 
 * 
 * @author Trevor Decker
 * 
 * The controller class for tracking how the buggy actually hidden state allowing for simulated 
 * sensors to receive information  
 * y is latitude, x is longitude
 */
public final class SimulatedBuggy {
	private static SimulatedBuggy instance;
	private boolean brakesDown = false;
	private Publisher simPosePub = new Publisher(NodeChannel.SIM_POSE.getMsgPath());
	
	/**
	 * @return the brakesDown
	 */
	public boolean isBrakesDown() {
		return brakesDown;
	}

	/**
	 * @param brakesDown the brakesDown to set
	 */
	public void setBrakesDown(boolean brakesDown) {
		this.brakesDown = brakesDown;
	}

	private double  x = 0.0;
	private double  y = 0.0;
	private double  th = 0.0;
	private double wheelTh = 0.0;//the orintation in buggy coordinates of the front wheel 
	private double dx = 0.0;
	private double dy = 0.0;
	private double dth = 0.0;
	private long lastUpdateTime;
	
	/**
	 * 
	 * @return a reference to the simulated buggy
	 */
	public static synchronized SimulatedBuggy getInstance(){
		if(instance == null){
			instance = new SimulatedBuggy();
		}
		return instance;
	}
	
	/**
	 * Constructor for the simulated buggy
	 */
	private SimulatedBuggy() {
		//set init values
		x = 0.0;
		y = 0.0;
		th = 0.0; //in degrees
		dx = 0.0;
		dy = 0.0;
		dth = 0.0; // in degrees
		lastUpdateTime = new Date().getTime();
		
		java.util.Timer t = new java.util.Timer();
		t.schedule(new TimerTask() {
			//TODO add delays and error 

		            @Override
		            public void  run() {
		            	//figure out timing 
		            	long now = new Date().getTime();
		            	long dtMili = now - lastUpdateTime;
		            	lastUpdateTime = now;
		            	double dt = dtMili/100.0;
		            	double heading = Util.normalizeAngleDeg(wheelTh + th + dth);
		            	double headingRad = Math.toRadians(heading);
		            	//now update the internal state
		            	x = x +dx*Math.cos(headingRad)*dt -dy*Math.sin(headingRad)*dt+0;//(2*Math.random()-1)/10;
		            	y = y +dx*Math.sin(headingRad)*dt+dy*Math.cos(headingRad)*dt+0;//(2*Math.random()-1)/10;
		            	th = heading;
		            	//TODO make pose message periodic 
		            	simPosePub.publish(new GPSPoseMessage(new Date(), y, x, th));

		            	
		            }
		        }, 1000, 50);
	}
	
	/**
	 * Getter for the wheel angle 
	 * @return the current wheel value
	 */
	public double getWheelTh(){
		return wheelTh;
		
	}
	
	/**
	 * updates the orientation of the front wheel 
	 * @param newWheelTh the orientation of the front wheel
	 */
	public void setWheelTh(double newWheelTh){
		wheelTh = newWheelTh;
	}

	/**
	 * @return the x
	 */
	public double getX() {
		return x;
	}

	/**
	 * @param x the x to set
	 */
	public void setX(double x) {
		this.x = x;
	}

	/**
	 * @return the y
	 */
	public double getY() {
		return y;
	}

	/**
	 * @param y the y to set
	 */
	public void setY(double y) {
		this.y = y;
	}

	/**
	 * @return the current th value in degrees
	 */
	public double getTh() {
		return th;
	}

	/**
	 * @param th the th to set in degrees
	 */
	public void setTh(double th) {
		this.th = th;
	}

	/**
	 * @return the dx
	 */
	public double getDx() {
		return dx;
	}

	/**
	 * @param dx the dx to set
	 */
	public void setDx(double dx) {
		this.dx = dx;
	}

	/**
	 * @return the dy
	 */
	public double getDy() {
		return dy;
	}

	/**
	 * @param dy the dy to set
	 */
	public void setDy(double dy) {
		this.dy = dy;
	}

	/**
	 * @return the dth in degrees
	 */
	public double getDth() {
		return dth;
	}

	/**
	 * @param dth the dth to set in degrees
	 */
	public void setDth(double dth) {
		this.dth = dth;
	}
	
	
}
