package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.ui.Gui;

public class main {
	static Robot buggy;

	public static void main(String args[]) {
		// ArrayList<Integer> cameras = new ArrayList<Integer>(); //TODO have
		// this set the cameras to use
		config.getInstance();// must be run at least once
		for (int i = 0; i < args.length; i++) {
			/*
			 * if (args[i].equalsIgnoreCase("-c")) { if (i+1 < args.length) {
			 * cameras.add(Integer.valueOf(args[1+i++])); } } else
			 */
			if (args[i].equalsIgnoreCase("-g")) {
				config.getInstance().GUI_ON = false;
			} else if (args[i].equalsIgnoreCase("+g")) {
				config.getInstance().GUI_ON = true;
			} else if (args[i].equalsIgnoreCase("-r")) {
				config.getInstance().active = false;
			} else if (args[i].equalsIgnoreCase("+r")) {
				config.getInstance().active = true;
			}
		}

		if (config.GUI_ON) {
			Gui.getInstance();
		}
		// starts the robot
		if (config.DATA_PLAY_BACK_DEFAULT) {
			SimRobot sr = new SimRobot();
		} else {
			Robot.getInstance();
		}

		// Keep this thread alive by spinning.
		while (true) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

}
