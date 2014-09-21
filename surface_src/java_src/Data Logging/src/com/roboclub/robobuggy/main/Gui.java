package com.roboclub.robobuggy.main;

import java.awt.GridLayout;

import javax.swing.JFrame;

public class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;
	
	private final int CAMERA_ID = 0;
	
	private static JFrame window;
	private static CameraPanel cameraPanel;
	private static GpsPanel gpsPanel;
	private static ArduinoPanel arduinoPanel;
	private static ControlsPanel controlsPanel;
	private static ImuPanel imuPanel;
	
	public Gui() {
		window = new JFrame();
		window.setSize(800, 600);
		window.setTitle("RoboBuggy Data Gathering");
		window.setLayout(new GridLayout(2, 2, 0, 0));
		
		// Initialize Panels for Window
		try {
			cameraPanel = new CameraPanel( CAMERA_ID );
			//gpsPanel = new GpsPanel();
			arduinoPanel = new ArduinoPanel();
			/*controlsPanel = new ControlsPanel();
			imuPanel = new ImuPanel();*/
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
		
		// Add panels to window
		window.add(cameraPanel);
		//window.add(gpsPanel);
		window.add(arduinoPanel);
		/*window.add(controlsPanel);
		window.add(imuPanel);*/
		
		window.setVisible(true);
		window.setResizable(false);
	}
	
	private void closeWindow(int exitCode) {
		//gpsPanel.closePort();
    	//arduinoPanel.closePort();
    	//imuPanel.closePort();
    	cameraPanel.close();
    	
        System.exit(exitCode);
	}
	
	@SuppressWarnings("unused")
	public static void main(String args[]) {		
		Gui gui = new Gui();
	}
}