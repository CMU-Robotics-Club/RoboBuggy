package com.roboclub.robobuggy.main;

import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.Logger;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

import com.roboclub.robobuggy.logging.RobotLogger;

public class CameraPanel extends JPanel {
	private static final long serialVersionUID = 2045798342979823126L;
	/* Panel Dimensons */
	private static final int PANEL_WIDTH = 400;
	private static final int PANEL_HEIGHT = 300;
	private static final int WIDTH = 320;
	private static final int HEIGHT = 240;
	private static final Size size = new Size(WIDTH,HEIGHT);
	
	private VideoCapture camera;
	private CameraThread cameraFeed;
	private BufferedImage image = null;
	
	public CameraPanel( int cameraId ) throws Exception {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		this.setSize(PANEL_WIDTH, PANEL_HEIGHT);
		this.setLayout(new FlowLayout(FlowLayout.CENTER, 0, 0));
		
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
		g.drawImage(this.image, (PANEL_WIDTH-WIDTH)/2, (PANEL_HEIGHT-HEIGHT)/2, this);
	}
	
	public void redraw() {
		this.repaint();
	}

	SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
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
			
				// Log and hope we log quickly
				// message is now contained in tmp
			    RobotLogger rl = RobotLogger.getInstance();
			    Date now = new Date();
			    long time_in_millis = now.getTime();
			    rl.sensor.logImage(time_in_millis, "", image);	
				redraw();
			}
		}
		
		public void close() {
			this.running = false;
		}
	}	
}