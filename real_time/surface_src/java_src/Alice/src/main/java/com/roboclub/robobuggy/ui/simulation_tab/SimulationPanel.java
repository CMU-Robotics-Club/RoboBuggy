package com.roboclub.robobuggy.ui.simulation_tab;

import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ui.RobobuggyGUIContainer;

/**
 * This class is for providing information about what the simulator actual state is and what the
 * state estimator/path planner thinks the state of the system is
 *
 * @author Trevor Decker
 */
public class SimulationPanel extends RobobuggyGUIContainer {

    /**
     * the constructor for the simulation panel
     */
    public SimulationPanel() {
        addComponent(new PoseViewer(NodeChannel.SIM_POSE, false), 0.0, 0.0, 0.5, 1.0);
        addComponent(new PoseViewer(NodeChannel.POSE, true), 0.5, 0.0, 0.5, 1.0);


    }

}
