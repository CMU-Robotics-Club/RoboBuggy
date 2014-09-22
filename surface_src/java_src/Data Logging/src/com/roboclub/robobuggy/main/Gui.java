package com.roboclub.robobuggy.main;

import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

public final class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;
	
	private final int CAMERA_ID = 1;
	private static Gui instance = null;
	private static final int WIDTH = 900;
	private static final int HEIGHT = 800;

	private static JFrame window;
	private static GpsPanel gpsPanel;
	private static ArduinoPanel arduinoPanel;
	private static ControlsPanel controlsPanel;
	private static CameraPanel cameraPanel;
	private static ImuPanel imuPanel;
	private static boolean playPauseState;

	public static Gui getInstance(){
		if(instance == null){
			System.out.println("Created Gui instance");
			instance = new Gui();
		}
		return instance;
	}
	
	public Gui() {
		playPauseState = false;
	}
	
	public void populate(){
		window = new JFrame();
		window.setTitle("RoboBuggy Data Gathering");
		
		// Initialize Panels for Window
		try {
			gpsPanel = new GpsPanel();
			arduinoPanel = new ArduinoPanel();
			controlsPanel = new ControlsPanel();
			imuPanel = new ImuPanel();
			//cameraPanel = new CameraPanel(CAMERA_ID);
			
			addPanels();
		} catch (Exception e) {
			e.printStackTrace();
			closeWindow(-1);
		}
		
		// Close ports and close window upon exit
		window.addWindowListener(new java.awt.event.WindowAdapter() {
		    @Override
		    public void windowClosing(java.awt.event.WindowEvent windowEvent) {
		    	closeWindow(0);
		    }
		});
		
		window.pack();
		window.setVisible(true);
	}
	
	private void addPanels() {
		Container pane = window.getContentPane();
		pane.setLayout(new GridBagLayout());
		pane.setSize(WIDTH, HEIGHT);
		GridBagConstraints c = new GridBagConstraints();
		
		JPanel rightPanel = new JPanel();
		rightPanel.setLayout(new GridBagLayout());
		c.fill = GridBagConstraints.BOTH;
		c.gridheight = 1;
		c.gridwidth = 1;
		c.gridy = 0;
		c.gridx = 0;
		c.weightx = 1;
		c.weighty = 0.5;
		rightPanel.add(controlsPanel, c);
		c.gridy = 1;
		rightPanel.add(gpsPanel, c);
		
		JPanel leftPanel = new JPanel();
		leftPanel.setLayout(new GridBagLayout());
		c.gridy = 1;
		c.weightx = 1;
		c.weighty = 0.25;
		leftPanel.add(arduinoPanel, c);
		c.gridy = 0;
		c.weighty = 0.75;
		leftPanel.add(imuPanel, c);		
		
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 0.66;
		c.weighty = 1;
		pane.add(leftPanel, c);
		c.gridx = 1;
		c.weightx = 0.34;
		pane.add(rightPanel, c);
	}
	
	private void closeWindow(int exitCode) {
		if (gpsPanel != null && gpsPanel.isConnected()) {
			gpsPanel.closePort();
		}
		if (arduinoPanel != null && arduinoPanel.isConnected()) {
			arduinoPanel.closePort();
		}
		if (imuPanel != null && imuPanel.isConnected()) {
			imuPanel.closePort();
		}
		
    	//cameraPanel.close();
    	
        System.exit(exitCode);
	}
	
	public static void main(String args[]) {
		Gui.getInstance().populate();
		
		/*try
		{
			Process p = Runtime.getRuntime().exec("C:\\Users\\abc\\buggy-log\\VisionSystem.exe");
		}catch(Exception exc){}*/
	}


	public static boolean GetPlayPauseState() {
		return playPauseState;
	}


	public static void setPlayPauseState(boolean playPauseState) {
		Gui.playPauseState = playPauseState;
	}
}