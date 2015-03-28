package com.roboclub.robobuggy.nodes;

/**
 * 
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

import java.awt.BorderLayout;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.File;

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

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.sensors.SensorType;

public class VisionNode {
	private SensorType sensorType;
	private boolean connected;
	private SensorState state;
	
	// TODO Consider moving to array for exandability
	private VideoCapture frontFeed;
	private VideoCapture rearFeed;
	private VideoCapture overLookFeed;
	private CameraPanel frontPanel;
	private CameraPanel rearPanel;
	private CameraPanel overLookPanel;
	
	public VisionNode(SensorChannel sensor) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		try {
			Process process = new ProcessBuilder("C:\\Users\\Robot\\Documents\\GitHub\\RoboBuggy\\surface_src\\java_src\\Alice\\jni\\VisionSystem.exe",
					"-c",String.valueOf(config.FRONT_CAM_INDEX)).start();
			Process p2 = new ProcessBuilder("C:\\Users\\Robot\\Documents\\GitHub\\RoboBuggy\\surface_src\\java_src\\Alice\\jni\\VisionSystem.exe","-c",String.valueOf(config.REAR_CAM_INDEX)).start();
		} catch (Exception e) {
			e.printStackTrace(); 
		}
		
		
		this.sensorType = SensorType.VISION;
		this.connected = false;
		
		/*if(!initCameras()) return;
		
		connected = true;
		if(config.FRONT_CAM_ON){
			frontPanel = new CameraPanel("FRONT", frontFeed);
		}
		if(config.REAR_CAM_ON){
			rearPanel = new CameraPanel("REAR", rearFeed);
		}
		
		if(config.OVERLOOK_CAM_ON){
			overLookPanel = new CameraPanel("OVERLOOK",overLookFeed);
		}*/
	}

	private boolean initCameras() {
		if(config.FRONT_CAM_ON){
			frontFeed = new VideoCapture(config.FRONT_CAM_INDEX);
		}
		
		if(config.REAR_CAM_ON){
			rearFeed = new VideoCapture(config.REAR_CAM_INDEX);
		}
		
		if(config.OVERLOOK_CAM_ON){
			overLookFeed = new VideoCapture(config.OVERLOOK_CAM_INDEX);
		}
		
		try {
			Thread.sleep(1000);
		} catch (Exception e) {
			e.printStackTrace();
			return false;
		}
		
		if (config.FRONT_CAM_ON && !frontFeed.isOpened()) {
			System.out.println("Failed to open front camera: " + 
					config.FRONT_CAM_INDEX);
			return false;
		} else if (config.REAR_CAM_ON && !rearFeed.isOpened()) {
			System.out.println("Failed to open rear camera: " + 
					config.REAR_CAM_INDEX);
			return false;
		}else if(config.OVERLOOK_CAM_ON && !overLookFeed.isOpened()){
			System.out.println("Failed to open overlook camera: "+config.OVERLOOK_CAM_INDEX);
			return false;
		}
		
		//TODO move to a seperate folder in refactor 
		//creates a folder for each of the open videoStreams
	//	System.out.println(RobotLogger.logFolder.getAbsolutePath());
		//TODO make work
		
		return true;
	}
	
	
	public SensorState getState() {
		return this.state;
	}

	
	public boolean isConnected() {
		return this.connected;
	}

	
	public long timeOfLastUpdate() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public boolean close() {
		if (connected) {
			try {
				frontFeed.release();
			} catch (Exception e) {
				System.out.println("Failed to close front feed!");
			}
			
			try {
				rearFeed.release();
			} catch (Exception e) {
				System.out.println("Failed to close rear feed!");
			}
			
			frontPanel.setVisible(false);
			rearPanel.setVisible(false);
		}
		connected = false;
		state = SensorState.DISCONNECTED;
		
		return false;
	}

	 
	public SensorType getSensorType() {
		return this.sensorType;
	}

	public void setDisplay(boolean value) {
		if (connected) {
			frontPanel.setVisible(value);
			rearPanel.setVisible(value);
		    overLookPanel.setVisible(value);
		}
	}
	
	private class CameraPanel extends JPanel {
		private static final long serialVersionUID = 4784083162709347884L;

		private static final int WIDTH = 320;
		private static final int HEIGHT = 240;
		private BufferedImage img;
		
		public CameraPanel(String title, VideoCapture feed) {
			this.setLayout(new BorderLayout());
			
			Thread thread = new Thread(new Runnable() {
				Mat frame = new Mat();
				Mat dst = new Mat();
				MatOfByte mb = new MatOfByte();
				Size size = new Size(WIDTH, HEIGHT);
				
				@Override
				public void run() {
					while (connected) {
						try {
							feed.read(frame);
							Imgproc.resize(frame, dst, size);
							Highgui.imencode(".jpg", dst, mb);
							img = ImageIO.read(new ByteArrayInputStream(mb.toArray()));
							//TODO move to outside function this is such a hack 
							 File outputfile = new File("saved.jpg");//TODO save to folder and make unique number
							 ImageIO.write(img, "png", outputfile);
							repaint();
							
						} catch (Exception e) {
							e.printStackTrace();
							close();
							break;
						}
					}
					
					System.out.println("Vision Thread - " + title + " destroyed!");
				}
			});
			thread.start();
			
			JDialog window = new JDialog();
			window.setDefaultCloseOperation(JDialog.HIDE_ON_CLOSE);
			window.setTitle(title);
			window.setModal(false);
			window.setSize(WIDTH+5, HEIGHT+5);
			window.setResizable(false);
			
			window.add(this);
			window.setVisible(true);
		}
		
		public void paintComponent(Graphics g) {
			if (img != null) {
				g.drawImage(img, 0, 0, this);
			}
		}
	}
}
