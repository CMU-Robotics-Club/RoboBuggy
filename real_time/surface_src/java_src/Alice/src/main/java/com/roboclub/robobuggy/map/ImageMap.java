package com.roboclub.robobuggy.map;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ui.LocTuple;

public class ImageMap {
	private BufferedImage map;
	private ArrayList<LocTuple> locs;
	private LocTuple imgNorthWest;
	private LocTuple imgSouthEast;
	
	public ImageMap(){
	try {
		map = ImageIO.read(new File("images/lat_long_course_map.png"));
	} catch(Exception e) {
		new RobobuggyLogicNotification("Unable to read map image!", RobobuggyMessageLevel.WARNING);
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
	}

	/**
	 * @return the map
	 */
	public BufferedImage getMap() {
		return map;
	}

	/**
	 * @param map the map to set
	 */
	public void setMap(BufferedImage map) {
		this.map = map;
	}
	
	public void draw(Graphics g,int width,int height){
		Graphics2D g2d = (Graphics2D) g.create();
		g.drawImage(getMap(), 0, 0, width, height, Color.black, null);

		for (int i = 0; i < locs.size(); i++) {
			LocTuple mTuple = locs.get(i);
			drawTuple(g2d, mTuple,width,height);
		}
		g2d.dispose();
	}
	
	public void drawTuple(Graphics2D g2d, LocTuple mTuple,int width,int height){
//		double dx = imgSouthWest.getLatitude() - imgNorthEast.getLatitude();
//		double dy = imgSouthWest.getLongitude() - imgNorthEast.getLongitude();
//		double x = (mTuple.getLatitude() - imgNorthEast.getLatitude()) / dx * frameWidth;
//		double y = (mTuple.getLongitude() - imgSouthWest.getLongitude()) / dy * frameHeight;
		
		double dx = Math.abs(imgSouthEast.getLongitude() - imgNorthWest.getLongitude());
		double dy = Math.abs(imgSouthEast.getLatitude() - imgNorthWest.getLatitude());
		
		double latdiff = Math.abs(mTuple.getLatitude() - imgNorthWest.getLatitude());
		double londiff = Math.abs(mTuple.getLongitude() - imgNorthWest.getLongitude());
		
		double px = (londiff * width) / dx;
		double py = (latdiff * height) / dy;
		
		int cDiameter = 5;
		g2d.setColor(Color.RED);
		g2d.fillOval((int)px, (int)py, cDiameter, cDiameter);
	}
	
}
