package com.roboclub.robobuggy.main;

import java.util.ArrayList;

/**
 * @author Trevor Decker
 * @author Kevin Brennan
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: Determines what the buggy thinks it should do next 
 */

public class Planner implements Runnable {
	private boolean running;
	
	/**
	 * TODO 
	 */
	public Planner() {
		running = true;
	}
	
	/**
	 * TODO
	 */
	@Override
	public void run() {
		//currently a test of the servo, todo change this to a seprate test class 
		int i = 0;
		boolean up = true;
		while(running) {
			//Robot.WriteAngle(i);
			if (up) i+=5;
			else i-=5;
			if (i >= 90) up = false;
			else if (i <= 0) up = true;
			
			try {
				Thread.sleep(100);
			} catch (Exception e) {
				Thread.currentThread().interrupt();
			}
		}
		
		Robot.ShutDown();
	}
}
