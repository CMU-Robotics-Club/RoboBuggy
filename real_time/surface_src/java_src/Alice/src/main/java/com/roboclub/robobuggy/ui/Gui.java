package com.roboclub.robobuggy.ui;

import java.awt.Container;
import java.awt.event.WindowEvent;
import java.util.ArrayList;

import javax.swing.JFrame;

/**
 * 
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
	
	public enum GuiPubSubTopics {
		GUI_LOG_BUTTON_UPDATED,
	}
	public void fixPaint(){
		for(int i = 0;i<windowList.size();i++){
			windowList.get(i).repaint(); 
		}
	}
	
	
	 //added data panel to figure out adding data in real time to gui
	public static Gui getInstance() {
		if (instance == null) {
			instance = new Gui();
		}
		return instance;
	}

	//The windowList is a list of all of the windows that are a part of the gui
	ArrayList<RoboBuggyJFrame> windowList = new ArrayList<RoboBuggyJFrame>();
	
	// constructor for GUI this method sets up all of the gui elements that the system uses 
	public Gui() {
		System.out.println("Starting GUI");
		RoboBuggyJFrame mainWindow = new RoboBuggyJFrame("MainWindow",1.0,1.0);	
		AnalyticsPanel analyPane = new AnalyticsPanel();
		ControlPanel cntrlPane = new ControlPanel();
		//addComponent syntax is (newComponent,percentageLeft,percentageTop,percentageWidth,percentageHeight)
		mainWindow.addComponent(cntrlPane, 0.0, 0.0, .3, 1.0);
		mainWindow.addComponent(analyPane, 0.3, 0.0, .7, 1.0);
		//the repaint is needed for a non blank screen to appear when the gui is first displayed 
		//mainWindow.resize(0, 0);
		//mainWindow.fullScreenRepaint();
		mainWindow.repaint();
		windowList.add(mainWindow);
		
		
	}
	
	//This method closes all gui elements relating to this instance of robobuggy
	public static void close() {
		System.out.println("Trying to close the window!");
		instance.dispatchEvent(new WindowEvent(instance, WindowEvent.WINDOW_CLOSING));
		System.out.println("Apparently we've closed the window!");
	}

}