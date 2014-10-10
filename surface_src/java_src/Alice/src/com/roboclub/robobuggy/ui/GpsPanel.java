package com.roboclub.robobuggy.ui;

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.Date;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.map.Point;
import com.roboclub.robobuggy.map.Rect;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class GpsPanel extends JPanel {
	private static final long serialVersionUID = 1399590586061060311L;
	
	/* Image Dimensons */
	private static final int WIDTH = 400;
	private static final int HEIGHT = 270;
	
	private static final Point UR = new Point(-79.94010f, 40.442395f);
	private static final Point UL = new Point(-79.948686f, 40.442395f);
	private static final Point LL = new Point(-79.948686f, 40.438363f);
	private static final double lon_width  = UR.getX() - UL.getX();
	private static final double lat_height = UL.getY() - LL.getY();
	private static int pixelX = -1;
	private static int pixelY = -1;
	
	private static final Rect MAP_COORD = new Rect(UR, UL, LL);
	private Point currLoc;
	private BufferedImage map;
	
	/**
	 * Constructor for GPS panel on user interface. Initializes a serial communication
	 * for reading from serial port. For the ui, it creates a graph marking the last
	 * longitude and latitude read.
	 */
	public GpsPanel() {
		this.currLoc = new Point(0,0);
		this.setPreferredSize(new Dimension(WIDTH, HEIGHT));
		
		// Load map image as background
		try {
			map = ImageIO.read(new File("images/course_map.png"));
		} catch (Exception e) {
			System.out.println("Unable to open map background!");
		}
	}
	private boolean painted = false;
	@Override
	public void paintComponent(Graphics g) {
		//super.paintComponent(g);
		if (!painted)g.drawImage(map, 0, 0, WIDTH, HEIGHT, Color.black, null);
		
		if (pixelX > 0 && pixelY > 0) {
			g.setColor(Color.RED);
			g.fillOval(pixelX, pixelY, 5, 5);
		}
		
	}
	
	private void setPixels() {
		if (MAP_COORD.within(currLoc)) {
			pixelX = (int)(WIDTH * (currLoc.getX() - UL.getX()) / lon_width);
			pixelY = (int)(HEIGHT * (UL.getY() - currLoc.getY()) / lat_height);
			
			repaint();
		} else {
			pixelX = -1;
			pixelY = -1;
		}
		
	}
}