package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * This class is for providing information about what the simulator actual state is and what the
 *  state estimator/path planner thinks the state of the system is 
 * @author Trevor Decker
 *
 */
public class SimulationPanel extends RobobuggyGUIContainer{

	/**
	 * the constructor for the simulation panel 
	 */
	public SimulationPanel() {
		addComponent(new PoseViewer(NodeChannel.SIM_POSE), 0.0, 0.0, 0.5, 1.0);
		addComponent(new PoseViewer(NodeChannel.POSE), 0.5, 0.0, 0.5, 1.0);

	
	}
	
}
