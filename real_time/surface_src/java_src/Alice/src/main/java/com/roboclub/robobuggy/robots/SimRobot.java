package com.roboclub.robobuggy.robots;

import java.io.IOException;

import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.localizers.KfLocalizer;
import com.roboclub.robobuggy.nodes.planners.SweepNode;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.simulation.SimulatedBuggy;
import com.roboclub.robobuggy.simulation.SimulatedGPSNode;
import com.roboclub.robobuggy.simulation.SimulatedRBSMNode;
import com.roboclub.robobuggy.simulation.SimulationPlayer;

public class SimRobot extends AbstractRobot{

	/**
	 * Returns a reference to the one instance of the {@link Robot} object.
	 * If no instance exists, a new one is created.
	 * @return a reference to the one instance of the {@link Robot} object
	 */
	public static AbstractRobot getInstance() {
		if (instance == null) {
			instance =  new SimRobot();
		}
		return instance;
	}
	
	private SimRobot(){
		super();
		
		nodeList.add(new SweepNode(NodeChannel.PATH_PLANNER));
		nodeList.add(new HighTrustGPSLocalizer());
		nodeList.add(new SimulatedGPSNode());
		nodeList.add(new SimulatedRBSMNode());
		
		SimulatedBuggy simBuggy = SimulatedBuggy.GetInstance();
		simBuggy.setDx(1.0);

	}
}
