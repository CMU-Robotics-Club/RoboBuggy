package com.roboclub.robobuggy.simulation;

import java.sql.Timestamp;
import java.util.Date;
import java.util.TimerTask;

import sun.security.jca.GetInstance;

/**
 * 
 * 
 * @author Trevor Decker
 * 
 * The controller class for tracking how the buggy actually hidden state allowing for simulated 
 * sensors to receive information  
 *
 */
public final class SimulatedBuggy {
	private static SimulatedBuggy instance;
	private boolean brakesDown = false;
	
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
	private double dx = 0.0;
	private double dy = 0.0;
	private double dth = 0.0;
	private long lastUpdateTime;
	
	/**
	 * 
	 * @return a reference to the simulated buggy
	 */
	public static SimulatedBuggy getInstance(){
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
		th = 0.0;
		dx = 0.0;
		dy = 0.0;
		dth = 0.0;
		lastUpdateTime = new Date().getTime();
		
		java.util.Timer t = new java.util.Timer();
		t.schedule(new TimerTask() {

		            @Override
		            public void run() {
		            	//figure out timing 
		            	long now = new Date().getTime();
		            	long dtMili = now - lastUpdateTime;
		            	lastUpdateTime = now;
		            	double dt = dtMili/1000.0;
		            
		            	//now update the internal state
		            	x = x +dx*Math.cos(th)*dt + dy*Math.sin(th)*dt;
		            	y = y +dx*-Math.sin(th)*dt+dy*Math.cos(th)*dt;
		            	th = th +dth*dt;
		            	
		            }
		        }, 1000, 100);
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
	 * @return the th
	 */
	public double getTh() {
		return th;
	}

	/**
	 * @param th the th to set
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
	 * @return the dth
	 */
	public double getDth() {
		return dth;
	}

	/**
	 * @param dth the dth to set
	 */
	public void setDth(double dth) {
		this.dth = dth;
	}
	
	
}
