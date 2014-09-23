package com.roboclub.robobuggy.main;

import java.awt.BorderLayout;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;

import javax.imageio.ImageIO;
import javax.swing.JDialog;
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
	/* Panel Dimensons */
	private static final int WIDTH = 320;
	private static final int HEIGHT = 240;
	private static final Size size = new Size(WIDTH,HEIGHT);
	
	private VideoCapture camera;
	private CameraThread cameraFeed;
	private BufferedImage image = null;
	private JDialog popup;
	
	public CameraPanel( int cameraId ) throws Exception {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		this.setLayout(new BorderLayout());
		
		camera = new VideoCapture(cameraId);
		
		try {
			// Allow camera to initialize
			Thread.sleep(1000);	
		} catch (Exception e) {
			System.exit(-1);
		}
		
		camera.open(cameraId);
		if (!camera.isOpened()) {
			throw new Exception("Failed to Open Camera: "+ cameraId);
		}
		
		cameraFeed = new CameraThread();
		cameraFeed.start();
		
		popup = new JDialog();
		popup.add(this);
		popup.setTitle("Camera " + cameraId);
		popup.setModal(false);
		popup.setSize(WIDTH+5, HEIGHT+20);
		popup.setResizable(false);
		popup.setVisible(Gui.GetDisplayState());
	}
	
	public void close() {
		cameraFeed.close();
		camera.release();
		popup.dispose();
	}
	
	public CameraThread getFeed() {
		return this.cameraFeed;
	}
	
	public void paintComponent(Graphics g) {
		g.drawImage(this.image, 0, 0, this);
	}
	
	public void redraw() {
		this.repaint();
	}
	
	public void setLogging(boolean input) {
		this.cameraFeed.setLogging(input);
	}
	
	public void setDisplaying(boolean input) {
		if (this.cameraFeed != null) {
			this.cameraFeed.setDisplaying(input);
		}
		if (this.popup != null) {
			this.popup.setVisible(input);
		}
	}

	SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss-SSS");
	private class CameraThread extends Thread {
		private boolean running = true;
		private boolean logging = Gui.GetPlayPauseState();
		private boolean displaying = Gui.GetDisplayState();
		
		public void run() {
			Mat frame = new Mat();
			camera.read(frame);
			Mat dst = new Mat();
			MatOfByte mb = new MatOfByte();
			
			camera.read(frame);
			Imgproc.resize(frame, dst, size);
			Highgui.imencode(".jpg", dst, mb);
			
			while(running) {
				if (displaying || logging) {
					try {
						camera.read(frame);
						Imgproc.resize(frame, dst, size);
						Highgui.imencode(".jpg", dst, mb);
						image = ImageIO.read(new ByteArrayInputStream(mb.toArray()));
						
						if (displaying) redraw();
						if (logging) {
							// Log and hope we log quickly
							// message is now contained in tmp
						    /*RobotLogger rl = RobotLogger.getInstance();
						    Date now = new Date();
						    long time_in_millis = now.getTime();
						    rl.sensor.logImage(time_in_millis, df.format(now), image);*/
						}
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
				else {
					CameraThread.yield();
				}
			}
		}
		
		public void close() {
			this.running = false;
		}
		
		public void setLogging(boolean input) {
			this.logging = input;
		}
		
		public void setDisplaying(boolean input) {
			this.displaying = input;
		}
	}	
}