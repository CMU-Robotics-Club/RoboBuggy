package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.simulation.SimulatedBuggy;
import com.roboclub.robobuggy.simulation.SimulatedGPSNode;
import com.roboclub.robobuggy.simulation.SimulatedImuNode;
import com.roboclub.robobuggy.simulation.SimulatedRBSMNode;
import com.roboclub.robobuggy.ui.AutonomousPanel;
import com.roboclub.robobuggy.ui.ConfigurationPanel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.ImuPanel;
import com.roboclub.robobuggy.ui.MainGuiWindow;
import com.roboclub.robobuggy.ui.PathPanel;
import com.roboclub.robobuggy.ui.PoseGraphsPanel;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;
import com.roboclub.robobuggy.ui.SimulationPanel;

import java.io.FileNotFoundException;
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
	//	nodeList.add(new KfLocalizer(100));
		nodeList.add(new SimulatedImuNode(100));
		nodeList.add(new SimulatedGPSNode(500));
		nodeList.add(new SimulatedRBSMNode());
	/*	ArrayList<GpsMeasurement> wayPoints = new ArrayList<GpsMeasurement>();
		for(int i = 0;i<100;i++){
			wayPoints.add(new GpsMeasurement(0,-i));
		}
		*/
		try {
			ArrayList<GpsMeasurement> wayPoints =
					WayPointUtil.createWayPointsFromWaypointList(RobobuggyConfigFile.getWaypointSourceLogFile());
			nodeList.add(new WayPointFollowerPlanner(wayPoints));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		SimulatedBuggy simBuggy = SimulatedBuggy.getInstance();
		simBuggy.setY(LocalizerUtil.convertLatToMeters(40.441705));
		simBuggy.setX(LocalizerUtil.convertLonToMeters(-79.941585));
		simBuggy.setTh(-110);
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		simBuggy.setDx(10);


		nodeList.add(new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, RobobuggyConfigFile.LOG_FILE_LOCATION,
				NodeChannel.getLoggingChannels()));
		//simBuggy.setDth(1);
	//	simBuggy.setDth(0.10);
		
		
		//setup the gui 
		RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow",1.0,1.0);	
		Gui.getInstance().addWindow(mainWindow);
		RobobuggyGUITabs tabs = new RobobuggyGUITabs();
		mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
		tabs.addTab(new MainGuiWindow(), "Home");
	//	tabs.addTab(new PoseGraphsPanel(),"poses");
	//	tabs.addTab(new ImuPanel(),"IMU");
		tabs.addTab(new AutonomousPanel(),"Autonomous");
		tabs.addTab(new SimulationPanel(),"Simulation");
		tabs.addTab(new PathPanel(),"Path Panel");
		tabs.addTab(new ConfigurationPanel(), "Configuration");


	}
}
