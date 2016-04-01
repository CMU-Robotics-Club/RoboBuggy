package com.roboclub.robobuggy.robots;

import java.io.IOException;
import java.util.ArrayList;

import sun.applet.Main;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.simulation.SimulatedBuggy;
import com.roboclub.robobuggy.simulation.SimulatedGPSNode;
import com.roboclub.robobuggy.simulation.SimulatedImuNode;
import com.roboclub.robobuggy.simulation.SimulatedRBSMNode;
import com.roboclub.robobuggy.simulation.SimulationPlayer;
import com.roboclub.robobuggy.ui.AutonomousPanel;
import com.roboclub.robobuggy.ui.ConfigurationPanel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.ImuPanel;
import com.roboclub.robobuggy.ui.ImuVisualWindow;
import com.roboclub.robobuggy.ui.MainGuiWindow;
import com.roboclub.robobuggy.ui.PoseGraphsPanel;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;
import com.roboclub.robobuggy.ui.SimulationPanel;
import com.roboclub.robobuggy.ui.VelocityWindow;

import java.util.ArrayList;

/**
 * A robot file for a simulated robot that can be used for internal testing of nodes along simulated paths 
 * @author Trevor Decker
 *
 */
public final class SimRobot extends AbstractRobot{
    private static SimRobot instance;
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
	
		nodeList.add(new HighTrustGPSLocalizer());
		nodeList.add(new SimulatedImuNode(100));
		nodeList.add(new SimulatedGPSNode(500));
		nodeList.add(new SimulatedRBSMNode());
		ArrayList<GpsMeasurement> wayPoints = new ArrayList<GpsMeasurement>();
		for(int i = 0;i<100;i++){
			wayPoints.add(new GpsMeasurement(0,i));
		}
	//	nodeList.add(new WayPointFollowerPlanner(wayPoints));
		SimulatedBuggy simBuggy = SimulatedBuggy.getInstance();
		simBuggy.setDx(1);

		//simBuggy.setDth(1);
		simBuggy.setDth(1.0);
		
		
		//setup the gui 
		RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow",1.0,1.0);	
		Gui.getInstance().addWindow(mainWindow);
		RobobuggyGUITabs tabs = new RobobuggyGUITabs();
		mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
		tabs.addTab(new MainGuiWindow(), "Home");
		tabs.addTab(new VelocityWindow(), "Velocity");
		tabs.addTab(new PoseGraphsPanel(),"poses");
		tabs.addTab(new ImuPanel(),"IMU");
		tabs.addTab(new  AutonomousPanel(),"Autonomous");
		tabs.addTab(new SimulationPanel(),"Simulation");


	}
}
