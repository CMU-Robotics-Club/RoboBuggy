package main;

import javax.swing.JPanel;

import org.opencv.core.Core;
import org.opencv.highgui.VideoCapture;

public class CameraPanel extends JPanel {
	private static final long serialVersionUID = 2045798342979823126L;
	private VideoCapture camera;
	
	public CameraPanel( int cameraId ) throws Exception {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		initializeCamera(cameraId);
	}
	
	private void initializeCamera(int cameraNum) throws Exception {
		camera = new VideoCapture(cameraNum);
		
		try {
			// Allow camera to initialize
			Thread.sleep(1000);	
		} catch (Exception e) {
			System.exit(-1);
		}
		
		camera.open(cameraNum);
		if (!camera.isOpened()) {
			throw new Exception("Failed to Open Camera!");
		}
	}
	
	public void close() {
		camera.release();
	}

	//TODO implement thread to update gui
}