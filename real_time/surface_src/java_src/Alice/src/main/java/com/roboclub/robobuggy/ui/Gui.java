package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

import java.awt.Container;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import javax.swing.JFrame;
import java.awt.event.WindowEvent;
import java.util.ArrayList;

import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * {@link JFrame} used to represent the robobuggy gui
 * @author Trevor Decker
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: This class is the top level controller for the graphical user interface displayed to the user 
 */
public final class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;

	
	private static Gui instance;
	
	/**
	 * Enum of {@link Gui} publisher and subscriber topics
	 */
	public enum GuiPubSubTopics {
		GUI_LOG_BUTTON_UPDATED,
	}

	/**
	 * Repaints all objects in the {@link Gui}
	 */
	public void fixPaint(){
		for(int i = 0;i<windowList.size();i++){
			windowList.get(i).repaint(); 
		}
	}
	
	/**
	 * Returns a reference to the one instance of the {@link Gui}. 
	 * If none exists, one will be constructed.
	 * @return a reference to the one instance of the {@link Gui}
	 */
	public synchronized static Gui getInstance() {
		if (instance == null) {
			instance = new Gui();
		}
		return instance;
	}

	//The windowList is a list of all of the windows that are a part of the gui
	private ArrayList<RobobuggyJFrame> windowList = new ArrayList<RobobuggyJFrame>();
	private MainGuiWindow mainGuiWindow;
	
	/**
	 * Construct a new {@link Gui} object
	 */
	private Gui() {
		new RobobuggyLogicNotification("StartingGUI", RobobuggyMessageLevel.NOTE);
		RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow",1.0,1.0);	
		mainGuiWindow = new MainGuiWindow();
		RobobuggyGUITabs tabs = new RobobuggyGUITabs();
		tabs.addTab(mainGuiWindow, "Home");
		tabs.addTab(new NodeViewer(),"Nodes");
		tabs.addTab(new PoseGraphsPanel(),"poses");
		tabs.addTab(new ImuPanel(),"IMU");
		mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
		mainWindow.repaint();		
		windowList.add(mainWindow);
	}
	

	/**
	 * @return the main gui window
	 */
	public MainGuiWindow getMainGuiWindow() {
		return mainGuiWindow;
	}
	
	/**
	 * Closes the {@link Gui}
	 */
	public static void close() {
		new RobobuggyLogicNotification("trying to close gui", RobobuggyMessageLevel.NOTE);
		instance.dispatchEvent(new WindowEvent(instance, WindowEvent.WINDOW_CLOSING));
		new RobobuggyLogicNotification("gui has been closed", RobobuggyMessageLevel.NOTE);
	}

}