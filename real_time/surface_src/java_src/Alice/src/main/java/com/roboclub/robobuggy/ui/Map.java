package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

/**
 * the map, it plots points where we are based on gps
 */
public class Map extends JPanel {
	private BufferedImage map;
	private ArrayList<LocTuple> locs;
	private LocTuple imgNorthWest;
	private LocTuple imgSouthEast;


	/**
	 * makes the new map
	 */
	public Map(){
		try {
			map = ImageIO.read(new File("images/lat_long_course_map.png"));
		} catch(Exception e) {
			System.out.println("Unable to open map!");
		}
		imgNorthWest = new LocTuple(40.443946388131266, -79.95532877484377);
		imgSouthEast = new LocTuple(40.436597411027364, -79.93596322545625);
		locs = new ArrayList<LocTuple>();
	}


	/**
	 * @param newPoint the point to add to the map
	 */
	public void addPoint(LocTuple newPoint){
		locs.add(newPoint);
		this.repaint();
	}

	/**
	 * @param newPoint point to add
	 * @param orientation the orientation to put the arrow at
	 */
	public void addArrow(LocTuple newPoint,double orientation){
		//TODO
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2d = (Graphics2D) g.create();
		g.drawImage(map, 0, 0, getWidth(), getHeight(), Color.black, null);

		for (int i = 0; i < locs.size(); i++) {
			LocTuple mTuple = locs.get(i);
			drawTuple(g2d, mTuple);
		}
		System.out.println("painting:"+locs.size());
		g2d.dispose();
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
		
		double px = (londiff * getWidth()) / dx;
		double py = (latdiff * getHeight()) / dy;
		
		System.out.println("painting at: "+px+","+py);
		int cDiameter = 5;
		g2d.setColor(Color.RED);
		g2d.fillOval((int)px, (int)py, cDiameter, cDiameter);
	}
	
	
}
