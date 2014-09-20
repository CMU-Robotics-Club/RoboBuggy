package main;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

public class CameraPanel extends JPanel {
	private static final long serialVersionUID = 2045798342979823126L;
	private static final int WIDTH = 320;
	private static final int HEIGHT = 240;
	private static final Size size = new Size(WIDTH,HEIGHT);
	
	private VideoCapture camera;
	private CameraThread cameraFeed;
	private BufferedImage image = null;
	
	public CameraPanel( int cameraId ) throws Exception {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		this.setBounds(0, 0, WIDTH, HEIGHT);
		
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
		
		cameraFeed = new CameraThread();
		cameraFeed.start();
	}
	
	public void close() {
		cameraFeed.close();
		camera.release();
	}
	
	public void paintComponent(Graphics g) {
		g.drawImage(this.image, 0, 0, this);
	}
	
	public void redraw() {
		this.repaint();
	}

	private class CameraThread extends Thread {
		private boolean running = true;
		
		public void run() {
			Mat frame = new Mat();
			camera.read(frame);
			Mat dst = new Mat();
			MatOfByte mb = new MatOfByte();
			
			while(running) {
				camera.read(frame);
				Imgproc.resize(frame, dst, size);
				Highgui.imencode(".jpg", dst, mb);
				
				try {
					image = ImageIO.read(new ByteArrayInputStream(mb.toArray()));
				} catch (IOException e) {
					e.printStackTrace();
				}
				
				redraw();
			}
		}
		
		public void close() {
			this.running = false;
		}
	}	
}