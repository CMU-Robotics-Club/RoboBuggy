package main;

import javax.swing.JFrame;

public class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;
	
	private final int CAMERA_ID = 0;
	
	private static JFrame window;
	private static CameraPanel cameraPanel;
	private static GpsPanel gpsPanel;
	private static ArduinoPanel arduinoPanel;
	
	public Gui() {
		window = new JFrame();
		
		// Initialize Panels for Window
		cameraPanel = new CameraPanel( CAMERA_ID );
		gpsPanel = new GpsPanel();
		arduinoPanel = new ArduinoPanel();
		
		// Close ports and close window upon exit
		window.addWindowListener(new java.awt.event.WindowAdapter() {
		    @Override
		    public void windowClosing(java.awt.event.WindowEvent windowEvent) {
		    	gpsPanel.closePort();
		    	arduinoPanel.closePort();
		    	
		        System.exit(0);
		    }
		});
		
		// Add panels to window
		window.add(cameraPanel);
		window.add(gpsPanel);
		window.add(arduinoPanel);
		
		window.setVisible(true);
		window.setDefaultCloseOperation(EXIT_ON_CLOSE);
	}
	
	@SuppressWarnings("unused")
	public static void main(String args[]) {		
		Gui gui = new Gui();
	}
}