package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

public class GpsPanel extends JPanel {
	private class LocTuple {
		private double latitude;
		private double longitude;
		private LocTuple(double x, double y){
			this.latitude = x;
			this.longitude = y;
		}
		
		public double getLatitude(){
			return latitude;
		}
		public double getLongitude(){
			return longitude;
		}
	}	
	
	private static final long serialVersionUID = 42L;
	private ArrayList<LocTuple> locs;
	private LocTuple imgNorthWest;
	private LocTuple imgSouthEast;
	private BufferedImage map;
	private boolean setup;
	private int frameWidth;
	private int frameHeight;
	private Subscriber gpsSub;
	
	
	public GpsPanel(){
		locs = new ArrayList<LocTuple>();
		imgNorthWest = new LocTuple(40.443946388131266, -79.95532877484377);
		imgSouthEast = new LocTuple(40.436597411027364, -79.93596322545625);
		try {
			map = ImageIO.read(new File("images/lat_long_course_map.png"));
		} catch(Exception e) {
			System.out.println("Unable to open map!");
		}
		setup = false;
		
		gpsSub = new Subscriber(SensorChannel.GPS.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				double latitude = ((GpsMeasurement)m).latitude;
				double longitude = ((GpsMeasurement)m).longitude;
				//todo put mag based on dir
				if(((GpsMeasurement)m).west) {
					longitude = -longitude;
				}
				
				locs.add(new LocTuple(latitude, longitude));
//				int gpsSize = locs.size(); // This is new: looks locs.size
//				if (gpsSize > 20) {        // if size > 20, remove first object
//					locs.remove(0);
//				}
				GpsPanel.this.repaint();  // refresh screen
			}
		});		
		
//		locs.add(new LocTuple(40.440443, -79.9427212));
	}
	
	private void setup() {
		frameWidth = getWidth();
		frameHeight = getHeight();
	}
	
	private void drawTuple(Graphics2D g2d, LocTuple mTuple){
//		double dx = imgSouthWest.getLatitude() - imgNorthEast.getLatitude();
//		double dy = imgSouthWest.getLongitude() - imgNorthEast.getLongitude();
//		double x = (mTuple.getLatitude() - imgNorthEast.getLatitude()) / dx * frameWidth;
//		double y = (mTuple.getLongitude() - imgSouthWest.getLongitude()) / dy * frameHeight;
		
		double dx = Math.abs(imgSouthEast.getLongitude() - imgNorthWest.getLongitude());
		double dy = Math.abs(imgSouthEast.getLatitude() - imgNorthWest.getLatitude());
		
		double latdiff = Math.abs(mTuple.getLatitude() - imgNorthWest.getLatitude());
		double londiff = Math.abs(mTuple.getLongitude() - imgNorthWest.getLongitude());
		
		double px = (londiff * frameWidth) / dx;
		double py = (latdiff * frameHeight) / dy;
		
		System.out.println("printing an oval at " + px + ", " + py);
		
		int cDiameter = 5;
		g2d.setColor(Color.RED);
		g2d.fillOval((int)px, (int)py - 500, cDiameter, cDiameter);
	}
	
	@Override
	public void paintComponent(Graphics g) {
		setup();
		super.paintComponent(g);
		if (!setup){
			setup();
			setup = true;
		}
		Graphics2D g2d = (Graphics2D) g.create();

		g.drawImage(map, 0, 0, frameWidth, frameHeight, Color.black, null);

		for	(LocTuple mTuple : locs) {
			drawTuple(g2d, mTuple);
		}
		g2d.dispose();
	}
}