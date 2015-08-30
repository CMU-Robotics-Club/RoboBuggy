package org.roboclub.robobuggy.drivers;

import org.roboclub.robobuggy.main.Robot;

/**
 * 
 * @author ??
 * @version 0.0
 * TODO document 
 *
 */
public class testAuton implements Driver{
		private boolean running;

		/**
		 * TODO document 
		 */
		public testAuton() {
			running = true;
		}

		@Override
		/**
		 * TODO document 
		 */
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
