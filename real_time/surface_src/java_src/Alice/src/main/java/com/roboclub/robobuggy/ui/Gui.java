package com.roboclub.robobuggy.ui;

import java.awt.Container;
import java.awt.event.WindowEvent;
import java.util.ArrayList;

import javax.swing.JFrame;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

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
	private Container pane;
	private ControlPanel ctrlPanel1;
	private ControlPanel ctrlPanel2;

	private AnalyticsPanel anlyPanel;
	
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
	public static Gui getInstance() {
		if (instance == null) {
			instance = new Gui();
		}
		return instance;
	}

	//The windowList is a list of all of the windows that are a part of the gui
	private ArrayList<RobobuggyJFrame> windowList = new ArrayList<RobobuggyJFrame>();
	
	/**
	 * Construct a new {@link Gui} object
	 */
	private Gui() {
		new RobobuggyLogicNotification("StartingGUI", RobobuggyMessageLevel.NOTE);
		RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow",1.0,1.0);	
		AnalyticsPanel analyPane = new AnalyticsPanel();
		ctrlPanel1 = new ControlPanel();
		//addComponent syntax is (newComponent,percentageLeft,percentageTop,percentageWidth,percentageHeight)
		mainWindow.addComponent(ctrlPanel1, 0.0, 0.0, .3, 1.0);
		mainWindow.addComponent(analyPane, 0.3, 0.0, .7, 1.0);
		//the repaint is needed for a non blank screen to appear when the gui is first displayed 
		//mainWindow.resize(0, 0);
		//mainWindow.fullScreenRepaint();
		mainWindow.repaint();
		windowList.add(mainWindow);
	}

	/**
	 * returns the control panel
	 * @return the control panel
	 */
	public ControlPanel getControlPanel() {
		return ctrlPanel1;
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