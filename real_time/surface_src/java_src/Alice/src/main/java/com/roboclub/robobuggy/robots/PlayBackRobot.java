package com.roboclub.robobuggy.robots;

import java.util.ArrayList;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.simulation.SensorPlayer;
import com.roboclub.robobuggy.simulation.SimulatedBuggy;
import com.roboclub.robobuggy.simulation.SimulatedGPSNode;
import com.roboclub.robobuggy.simulation.SimulatedRBSMNode;

public class PlayBackRobot extends AbstractRobot{
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
		SensorPlayer sp = new SensorPlayer(RobobuggyConfigFile.getPlayBackSourceFile(), 1);
	}
}
