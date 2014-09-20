package main;

import javax.swing.JFrame;

public class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;
	
	private final static int CAMERA_ID = 0;
	
	private static JFrame window;
	private static CameraPanel cameraPanel;
	private static GpsPanel gpsPanel;
	private static ArduinoPanel arduinoPanel;
	
	public static void main(String args[]) {
		window = new JFrame();
		
		// Initialize Panels for Window
		cameraPanel = new CameraPanel( CAMERA_ID );
		gpsPanel = new GpsPanel();
		arduinoPanel = new ArduinoPanel();
		
		initializeWindow();
	}
	
	public static void initializeWindow() {
		window.addWindowListener(new java.awt.event.WindowAdapter() {
		    @Override
		    public void windowClosing(java.awt.event.WindowEvent windowEvent) {
		    	gpsPanel.closePort();
		    	arduinoPanel.closePort();
		    	
		        System.exit(0);
		    }
		});
		
		window.add(cameraPanel);
		window.add(gpsPanel);
		window.add(arduinoPanel);
		
		window.setVisible(true);
		window.setDefaultCloseOperation(EXIT_ON_CLOSE);
	}
}