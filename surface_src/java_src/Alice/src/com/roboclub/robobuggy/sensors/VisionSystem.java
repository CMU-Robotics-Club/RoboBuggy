package com.roboclub.robobuggy.sensors;

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

import com.roboclub.robobuggy.main.config;

public class VisionSystem implements Sensor {
	private SensorType sensorType;
	private boolean connected;
	private SensorState state;
	
	// TODO Consider moving to array for exandability
	private VideoCapture frontFeed;
	private VideoCapture rearFeed;
	private CameraPanel frontPanel;
	private CameraPanel rearPanel;
	
	public VisionSystem(String string) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		this.sensorType = SensorType.VISION;
		this.connected = false;
		
<<<<<<< HEAD
		if(!initCameras()) return;
=======
		String frontCamIndex_str = Integer.toString(config.FRONT_CAM_INDEX);
		String rearCamIndex_str = Integer.toString(config.REAR_CAM_INDEX);
		Process externalProcess = new ProcessBuilder(config.VISION_SYSTEM_EXECUTABLE_LOCATION,"-c",frontCamIndex_str,"-c",rearCamIndex_str).start();
		//Process externalProcess = new ProcessBuilder(config.VISION_SYSTEM_EXECUTABLE_LOCATION,"-c",frontCamIndex_str).start();
>>>>>>> parent of 488f6e0... stable for rolls 10/25/14
		
		connected = true;
		frontPanel = new CameraPanel("FRONT", frontFeed);
		rearPanel = new CameraPanel("REAR", rearFeed);
	}

	private boolean initCameras() {
		frontFeed = new VideoCapture(config.FRONT_CAM_INDEX);
		rearFeed = new VideoCapture(config.REAR_CAM_INDEX);
		
		try {
			Thread.sleep(1000);
		} catch (Exception e) {
			e.printStackTrace();
			return false;
		}
		
		if (!frontFeed.isOpened()) {
			System.out.println("Failed to open front camera: " + 
					config.FRONT_CAM_INDEX);
			return false;
		} else if (!rearFeed.isOpened()) {
			System.out.println("Failed to open rear camera: " + 
					config.REAR_CAM_INDEX);
			return false;
		}
		
		return true;
	}
	
	@Override
	public SensorState getState() {
		return this.state;
	}

	@Override
	public boolean isConnected() {
		return this.connected;
	}

	@Override
	public long timeOfLastUpdate() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
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

	@Override
	public boolean reset() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public SensorType getSensorType() {
		return this.sensorType;
	}

	@Override
	public void publish() {
		// TODO Auto-generated method stub
	}

	public void setDisplay(boolean value) {
		if (connected) {
			frontPanel.setVisible(value);
			rearPanel.setVisible(value);
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
