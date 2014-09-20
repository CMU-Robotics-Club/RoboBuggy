package main;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;
	
	private static final int CAMERA_ID = 0;
	
	private static JFrame window;
	private static JPanel cameraPanel;
	private static JPanel gpsPanel;
	private static JPanel imuPanel;
	
	public static void main(String[] args) {
		window = new JFrame();
		window.setVisible(true);
		
		// Initialize Panels for Window
		cameraPanel = new CameraPanel(CAMERA_ID);
		gpsPanel = new GpsPanel();
	   	
		window.add(cameraPanel);
		//window.add(gpsPanel);
		//window.add(imuPanel);
	}
}