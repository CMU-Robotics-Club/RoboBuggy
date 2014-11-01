package com.roboclub.robobuggy.sensors;

import java.io.File;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

public class Camera implements Runnable {
	private int id;
	private boolean show;
	private boolean record;
	private VideoCapture camera;
	private boolean connected;
	private final Size size = new Size(320, 240);
	
	public Camera(int id, boolean show, boolean record) {
		this.id = id;
		this.show = show;
		this.record = record;
		
		// Set camera by id
		this.camera = new VideoCapture(id);
		
		try {
			Thread.sleep(5000);
		} catch (Exception e) {
			e.printStackTrace();
			camera = null;
			return;
		}
		
		// Open feed to camera
		camera.open(id);
		if (!camera.isOpened()) {
			System.out.println("Failed to connect to camera: " + id);
			camera = null;
			return;
		}
		
		connected = true;
	}
	
	public void setRecording(String dirName, boolean record) {
		if (record) {
			File dir = new File(dirName);
			if (!dir.exists()) dir.mkdirs();
			
			int run = 0;
			
			String filename = dirName + "camera" + this.id + "-run";
			File file = new File(filename + run + ".avi");
			
			while (file.exists()) {
				run++;
				file = new File(filename + run + ".avi");
			}
			
			this.record = true;
		} else {
		}
	}

	public void disconnect() {
		if (connected) {
			connected = false;
			record = false;
			show = false;
			camera.release();
		}
	}
	
	@Override
	public void run() {
		Mat frame = new Mat();
		
		while(connected) {
			if (record || show) {
				camera.read(frame);
				Imgproc.resize(frame, frame, size);
				
				if (record) {
					// TODO implement writing to video file
				}
				if (show) {
					// TODO implement display
				}
				
			} else {
				try {
					Thread.sleep(5000);
				} catch (Exception e) {
					e.printStackTrace();
					connected = false;
					// TODO Decide whether to reconnect
				}
			}
		}
		
		System.out.println("Camera " + this.id + " disconnected!");
	}
}
