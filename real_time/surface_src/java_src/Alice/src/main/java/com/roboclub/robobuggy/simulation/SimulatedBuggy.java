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
 */
public class SimulatedBuggy {
	private static SimulatedBuggy instance;
	
	private double  x = 0.0;
	private double  y = 0.0;
	private double  th = 0.0;
	private double dx = 0.0;
	private double dy = 0.0;
	private double dth = 0.0;
	private long lastUpdateTime;
	
	public static SimulatedBuggy GetInstance(){
		if(instance == null){
			instance = new SimulatedBuggy();
		}
		return instance;
	}
	
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
		            	long dt_mili = now - lastUpdateTime;
		            	lastUpdateTime = now;
		            	double dt = dt_mili/1000.0;
		            
		            	//now update the internal state
		            	x = x +dx*dt;
		            	y = y +dy*dt;
		            	th = th +dth*dt;
		            	
		            }
		        }, 0, 1000);
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
