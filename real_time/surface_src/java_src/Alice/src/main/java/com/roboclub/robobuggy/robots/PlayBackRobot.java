package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.simulation.SensorPlayer;
import com.roboclub.robobuggy.ui.AutonomousPanel;
import com.roboclub.robobuggy.ui.ConfigurationPanel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.MainGuiWindow;
import com.roboclub.robobuggy.ui.PathPanel;
import com.roboclub.robobuggy.ui.PoseGraphsPanel;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;

/**
 * Runs playback
 * @author Trevor Decker
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
	
		new SensorPlayer(RobobuggyConfigFile.getPlayBackSourceFile(), 1);
		RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow",1.0,1.0);	
		Gui.getInstance().addWindow(mainWindow);
		RobobuggyGUITabs tabs = new RobobuggyGUITabs();
		mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
		tabs.addTab(new MainGuiWindow(), "Home");
		tabs.addTab(new PoseGraphsPanel(),"poses");
		tabs.addTab(new  AutonomousPanel(),"Autonomous");
		tabs.add(new PathPanel(), "Path Visualizer");
		tabs.addTab(new ConfigurationPanel(),"Configuration");
	}
}	

