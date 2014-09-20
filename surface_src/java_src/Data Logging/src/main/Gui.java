package main;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;
	
	private final int CAMERA_ID = 0;
	
	private static JFrame window;
	private static JPanel cameraPanel;
	private static JPanel gpsPanel;
	private static JPanel encPanel;
	private static JPanel imuPanel;
	private static JPanel wheelPanel;
	
	public void main() {
		window = new JFrame();
		
		// Initialize Panels for Window
		cameraPanel = new CameraPanel( CAMERA_ID );
		
		window.add(cameraPanel);
		window.add(gpsPanel);
		window.add(encPanel);
		window.add(imuPanel);
		window.add(wheelPanel);
	}
}