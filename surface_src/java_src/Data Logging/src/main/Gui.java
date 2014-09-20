package main;

import javax.swing.JFrame;

public class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;
	
	private final static int CAMERA_ID = 0;
	
	private static JFrame window;
	private static CameraPanel cameraPanel;
	private static GpsPanel gpsPanel;
	
	public static void main(String args[]) {
		window = new JFrame();
		
		// Initialize Panels for Window
		cameraPanel = new CameraPanel( CAMERA_ID );
		gpsPanel = new GpsPanel();
		
		window.add(cameraPanel);
		window.add(gpsPanel);
		
		window.setVisible(true);
	}
}