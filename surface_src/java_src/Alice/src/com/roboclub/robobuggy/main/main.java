package com.roboclub.robobuggy.main;

import java.util.ArrayList;

public class main {
	static Robot buggy; 
	public static void main(String args[]) {
		ArrayList<Integer> cameras = new ArrayList<Integer>();
		boolean gui = true;
		boolean running = true;
		
		for (int i = 0; i < args.length; i++) {
			if (args[i].equalsIgnoreCase("-c")) {
				if (i+1 < args.length) {
					cameras.add(Integer.valueOf(args[1+i++]));
				}
			} else if (args[i].equalsIgnoreCase("-g")) {
				gui = false;
			} else if (args[i].equalsIgnoreCase("-r")) {
				running = false;
			}
		}
		
		//creates the robot
		Robot.getInstance();
	}
	
	
}
