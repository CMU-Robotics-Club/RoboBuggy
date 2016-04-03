package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.sensors.CameraNode;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.HillCrestImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ui.ConfigurationPanel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.ImuPanel;
import com.roboclub.robobuggy.ui.ImuVisualWindow;
import com.roboclub.robobuggy.ui.MainGuiWindow;
import com.roboclub.robobuggy.ui.PathPanel;
import com.roboclub.robobuggy.ui.PoseGraphsPanel;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;
import com.roboclub.robobuggy.ui.VelocityWindow;

/**
 * A robot class for doing data collection only with the live robot, will not attempt to autonomously drive 
 * @author Trevor Decker
 *
 */
public final class TransistorDataCollection extends AbstractRobot {
	private static TransistorDataCollection instance;
	private static final int ARDUINO_BOOTLOADER_TIMEOUT = 2000;

	/**
	 * Returns a reference to the one instance of the {@link Robot} object.
	 * If no instance exists, a new one is created.
	 * @return a reference to the one instance of the {@link Robot} object
	 */
	public static AbstractRobot getInstance() {
		if (instance == null) {
			instance =  new TransistorDataCollection();
		}
		return instance;
	}

	private TransistorDataCollection(){
		super();
		try {
			Thread.sleep(ARDUINO_BOOTLOADER_TIMEOUT);
		} catch (InterruptedException e) {
			new RobobuggyLogicNotification("Couldn't wait for bootloader, shutting down", RobobuggyMessageLevel.EXCEPTION);
			shutDown();
		}
		new RobobuggyLogicNotification("Logic Exception Setup properly" ,  RobobuggyMessageLevel.NOTE);
		// Initialize Nodes
		nodeList.add(new GpsNode(NodeChannel.GPS, RobobuggyConfigFile.getComPortGPS()));
		nodeList.add(new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, RobobuggyConfigFile.LOG_FILE_LOCATION,
				NodeChannel.getLoggingChannels()));
		nodeList.add(new RBSMNode(NodeChannel.ENCODER, NodeChannel.STEERING, RobobuggyConfigFile.getComPortRBSM(),
				RobobuggyConfigFile.RBSM_COMMAND_PERIOD));
		nodeList.add(new CameraNode(NodeChannel.PUSHBAR_CAMERA, 100));
		nodeList.add(new HillCrestImuNode());
		nodeList.add(new HighTrustGPSLocalizer());

		//setup the gui 
		RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow",1.0,1.0);	
		Gui.getInstance().addWindow(mainWindow);
		RobobuggyGUITabs tabs = new RobobuggyGUITabs();
		mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
		tabs.addTab(new MainGuiWindow(), "Home");
		tabs.addTab(new ImuVisualWindow(), "IMU");
		tabs.addTab(new VelocityWindow(), "Velocity");
		tabs.addTab(new PoseGraphsPanel(),"poses");
		tabs.addTab(new ImuPanel(),"IMU");
		tabs.addTab(new PathPanel(),"Path Panel");
		tabs.addTab(new ConfigurationPanel(),"Configuration");

	}
}

