package com.roboclub.robobuggy.main;

import java.util.ArrayList;

public class Planner implements Runnable {
	private boolean running;

	public Planner() {
		running = true;
	}

	@Override
	public void run() {
		int i = 0;
		boolean up = true;
		while (running) {
			// Robot.WriteAngle(i);
			if (up)
				i += 5;
			else
				i -= 5;
			if (i >= 90)
				up = false;
			else if (i <= 0)
				up = true;

			try {
				Thread.sleep(100);
			} catch (Exception e) {
				Thread.currentThread().interrupt();
			}
		}

		Robot.ShutDown();
	}
}
