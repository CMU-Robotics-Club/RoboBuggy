package tests.testActuators;

import com.roboclub.robobuggy.main.Robot;

/**
 * 
 * @author Trevor Decker
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: tester class for the servo
 */

public class TestServo {
	private static boolean running;

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		running = true;
		LeftRightSweep();

	}

	public static void LeftRightSweep() {
		// currently a test of the servo, todo change this to a seprate test
		// class
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
