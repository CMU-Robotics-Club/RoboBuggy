package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.WindowEvent;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JButton;
import javax.swing.JFrame;

import com.roboclub.robobuggy.main.Robot;

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

	 //added data panel to figure out adding data in real time to gui
	public static Gui getInstance() {
		if (instance == null) {
			instance = new Gui();
		}
		return instance;
	}

	//The windowList is a list of all of the windows that are a part of the gui
	ArrayList<RoboBuggyJFrame> windowList = new ArrayList<RoboBuggyJFrame>();
	
	public Gui() {
		System.out.println("Starting GUI");
		RoboBuggyJFrame mainWindow = new RoboBuggyJFrame("MainWindow",1.0,1.0);	
		AnalyticsPanel analyPane = new AnalyticsPanel();
		ControlPanel cntrlPane = new ControlPanel();
		mainWindow.addComponet(cntrlPane, 0.0, 0.0, .2, 1.0);
		mainWindow.addComponet(analyPane, 0.2, 0.0, .8, 1.0);
		windowList.add(mainWindow);
		
	}
	
	//This method closes all gui elements relating to this instance of robobuggy
	public static void close() {
		System.out.println("Trying to close the window!");
		instance.dispatchEvent(new WindowEvent(instance, WindowEvent.WINDOW_CLOSING));
		System.out.println("Apparently we've closed the window!");
	}

	/*
	public void populate() {
		this.setTitle("RoboBuggy Interface");
		this.setExtendedState(Frame.MAXIMIZED_BOTH);
		pane = this.getContentPane();
		pane.setLayout(new GridBagLayout());
		pane.setBackground(Color.DARK_GRAY);

		
		// Close ports and close window upon exit
		this.addWindowListener(new java.awt.event.WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent windowEvent) {
				try {
					Robot.ShutDown();
				}
				catch(NullPointerException e) {
					e.printStackTrace();
				}
			}
		});
		//frame needs to be visible for it to have a widht and height
		this.setVisible(true); 

		ctrlPanel1 = new ControlPanel();
		ctrlPanel2 = new ControlPanel();

		anlyPanel = new AnalyticsPanel();
		//GuiUtil.subPlot(1,1,0,0,0,0,anlyPanel,pane);
		//GuiUtil.subPlot(1,2,0,1,0,1,ctrlPanel2,pane);
		pane.add(new JButton("test"));

	
		this.pack();
	}
	
	@Override
	public void paint(Graphics g){
		int numComponets = this.getComponentCount();
		System.out.println("frame is being resized has:"+numComponets);
		for(int i = 0;i<numComponets;i++){
			this.getComponent(i).setBounds(0, 0, this.getWidth(), this.getHeight());;
		}

		super.paint(g);
	//	GuiUtil.subPlot(1,1,0,0,0,0,anlyPanel,pane);

		
	}
	
	public static void close() {
		System.out.println("Trying to close the window!");
		instance.dispatchEvent(new WindowEvent(instance, WindowEvent.WINDOW_CLOSING));
		System.out.println("Apparently we've closed the window!");
	}

	public static void EnableLogging() {
		instance.ctrlPanel1.enableLogging();
	}
	*/
}