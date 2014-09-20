package com.roboclub.robobuggy.ui;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

import org.opencv.highgui.VideoCapture;

public class CameraPanel extends JPanel {
	private static final long serialVersionUID = 2045798342979823126L;

	private static VideoCapture camera;	
	
	public static void initializeCamera(int camera_num) throws Exception {
		camera = new VideoCapture(camera_num);
		
		try {
			// Allow camera to initialize
			Thread.sleep(1000);	
		} catch (Exception e) {
			e.printStackTrace();
			throw new Exception("Unable to intialize camera object");
		}
		
		camera.open(0);
		if (!camera.isOpened()) {
			throw new Exception("Unable to Open Camera");
		}
	}
	
	
	public CameraPanel( int camera_id ) {
		JLabel cam_message = new JLabel("Cam STUFF goes here",SwingConstants.CENTER);
		this.add(cam_message);		// Read from camera forever
		/*Mat frame = new Mat();
		while(true) {
			camera.read(frame);
			
			MatOfByte bytemat = new MatOfByte();
			Highgui.imencode(".jpg", frame, bytemat);
			byte[] bytes = bytemat.toArray();
			InputStream in = new ByteArrayInputStream(bytes);
			
			try {
				BufferedImage img = ImageIO.read(in);
				camPanel.add( new JLabel( new ImageIcon(img) ) );
				camPanel.setVisible(true);
			} catch (Exception e) {
				e.printStackTrace();
				return;
			}
		}*/	
	}
}