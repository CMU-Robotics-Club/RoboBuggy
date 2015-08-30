package org.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

import org.roboclub.robobuggy.messages.GpsMeasurement;
import org.roboclub.robobuggy.ros.Message;
import org.roboclub.robobuggy.ros.MessageListener;
import org.roboclub.robobuggy.ros.SensorChannel;
import org.roboclub.robobuggy.ros.Subscriber;


/**
 * 
 * @author Kevin Brennan 
 *
 * @version 0.0
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class GpsPanel extends JPanel {
	/**
	 * TODO document
	 * @author Vasu
	 *
	 */
	private class LocTuple {
		private double latitude;
		private double longitude;
		/**
		 * TODO document
		 * @param x
		 * @param y
		 */
		private LocTuple(double x, double y){
			this.latitude = x;
			this.longitude = y;
		}
		
		/**
		 * TODO document
		 * @return
		 */
		public double getLatitude(){
			return latitude;
		}
		/**
		 * TODO document
		 * @return
		 */
		public double getLongitude(){
			return longitude;
		}
	}	
	
	private static final long serialVersionUID = 42L;
	private ArrayList<LocTuple> locs;
	private LocTuple imgNorthEast;
	private LocTuple imgSouthWest;
	private BufferedImage map;
	private boolean setup;
	private int frameWidth;
	private int frameHeight;
	private Subscriber gpsSub;
	
	/**
	 * TODO document
	 */
	public GpsPanel(){
		locs = new ArrayList<LocTuple>();
		imgNorthEast = new LocTuple(-79.93596322545625, 40.443946388131266);
		imgSouthWest = new LocTuple(-79.95532877484377, 40.436597411027364);
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
				locs.add(new LocTuple(latitude, longitude));
			}
		});		
		
		locs.add(new LocTuple(-79.94596322545625, 40.440946388131266));
	}
	
	/**
	 * TODO document
	 */
	private void setup() {
		frameWidth = getWidth();
		frameHeight = getHeight();
	}
	
	/**
	 * TODO document
	 * @param g2d
	 * @param mTuple
	 */
	private void drawTuple(Graphics2D g2d, LocTuple mTuple){
		double dx = imgSouthWest.getLatitude() - imgNorthEast.getLatitude();
		double dy = imgSouthWest.getLongitude() - imgNorthEast.getLongitude();
		double x = (mTuple.getLatitude() - imgNorthEast.getLatitude()) / dx * frameWidth;
		double y = (mTuple.getLongitude() - imgSouthWest.getLongitude()) / dy * frameHeight;
		int cDiameter = 5;
		g2d.setColor(Color.RED);
		g2d.drawOval((int)x, -(int)y, cDiameter, cDiameter);
		g2d.fillOval((int)x, -(int)y, cDiameter, cDiameter);
	}
	
	@Override
	/**
	 * TODO document
	 */
	public void paintComponent(Graphics g) {
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