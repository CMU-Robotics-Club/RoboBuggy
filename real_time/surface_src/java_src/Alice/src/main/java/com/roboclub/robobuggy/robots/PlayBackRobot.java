package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.simulation.SensorPlayer;

/**
 * Runs playback
 * @author trevordecker
 *
 */
public final class PlayBackRobot extends AbstractRobot{

    private static PlayBackRobot instance;
	/**
	 * Returns a reference to the one instance of the {@link Robot} object.
	 * If no instance exists, a new one is created.
	 * @return a reference to the one instance of the {@link Robot} object
	 */
	public static AbstractRobot getInstance() {
		if (instance == null) {
			instance =  new PlayBackRobot();
		}
		return instance;
	}
	
	private PlayBackRobot(){
		super();
		new SensorPlayer(RobobuggyConfigFile.getPlayBackSourceFile(), 1, RobobuggyConfigFile.getVideoFile());
	}
}
