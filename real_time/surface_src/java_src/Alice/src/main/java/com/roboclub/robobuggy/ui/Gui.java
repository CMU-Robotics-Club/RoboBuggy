package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

import javax.swing.JFrame;

import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.HashMap;

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


	//The windowList is a list of all of the windows that are a part of the gui
    HashMap<Integer, RobobuggyJFrame> windowMap;
	private int currentWindowId;
	private MainGuiWindow mainGuiWindow;

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
		windowMap.forEach((key,window) -> window.repaint());
	}
	
	/**
	 * Returns a reference to the one instance of the {@link Gui}. 
	 * If none exists, one will be constructed.
	 * @return a reference to the one instance of the {@link Gui}
	 */
	public static synchronized Gui getInstance() {
		if (instance == null) {
			instance = new Gui();
		}
		return instance;
	}

	/**
	 * Construct a new {@link Gui} object
	 */
	private Gui() {
		new RobobuggyLogicNotification("StartingGUI", RobobuggyMessageLevel.NOTE);
		windowMap= new HashMap<Integer, RobobuggyJFrame>();
		currentWindowId = -1;

	}
	
	
	/**
	 * Adds a new window to the 
	 * @param newWindow the window you want added
	 * @return a unique reference number to the window for access later 
	 */
	public synchronized int  addWindow(RobobuggyJFrame newWindow){
		currentWindowId++;
		windowMap.put(currentWindowId, newWindow);
		return currentWindowId;		
	}
	
	public synchronized void getWindow(int windowRefrence){
		windowMap.get(windowRefrence);
	}
	
	public synchronized void deleteWindow(int windowRefrence){
		windowMap.remove(windowRefrence);
		
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
	public static synchronized  void close() {
		new RobobuggyLogicNotification("trying to close gui", RobobuggyMessageLevel.NOTE);
		instance.dispatchEvent(new WindowEvent(instance, WindowEvent.WINDOW_CLOSING));
		new RobobuggyLogicNotification("gui has been closed", RobobuggyMessageLevel.NOTE);
		instance = null;
	}

}
