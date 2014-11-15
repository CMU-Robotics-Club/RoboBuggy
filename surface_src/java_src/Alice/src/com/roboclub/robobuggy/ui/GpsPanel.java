package com.roboclub.robobuggy.ui;

import java.awt.image.BufferedImage;
import java.io.File;
import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;

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
public class GpsPanel extends JPanel {
	private static final long serialVersionUID = 1399590586061060311L;
	
	/*private static final Point UR = new Point(-79.94010f, 40.442395f);
	private static final Point UL = new Point(-79.948686f, 40.442395f);
	private static final Point LL = new Point(-79.948686f, 40.438363f);
	private static final double lon_width  = UR.getX() - UL.getX();
	private static final double lat_height = UL.getY() - LL.getY();
	private static int pixelX = -1;
	private static int pixelY = -1;*/
	
	private MapPanel mapPanel;
	private JLabel lat, lon;
	
	/**
	 * Constructor for GPS panel on user interface. Initializes a serial communication
	 * for reading from serial port. For the ui, it creates a graph marking the last
	 * longitude and latitude read.
	 */
	public GpsPanel() {
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridBagLayout());
		
		GridBagConstraints gbc = new GridBagConstraints();
		gbc.gridx = 0;
		gbc.gridy = 0;
		gbc.fill = GridBagConstraints.BOTH;
		
		mapPanel = new MapPanel();
		this.add(mapPanel, gbc);
		
		JPanel lowerPanel = new JPanel();
		lowerPanel.setLayout(new GridLayout(1,4));
		
		lat = new JLabel();
		JLabel label = new JLabel("   Lat: ");
		lowerPanel.add(label);
		lowerPanel.add(lat);
		
		lon = new JLabel();
		label = new JLabel("   Lon: ");
		lowerPanel.add(label);
		lowerPanel.add(lon);
		
		gbc.gridy = 1;
		this.add(lowerPanel, gbc);
	}
	
	public void UpdatePos(Float lat_, Float lon_) {
		if (lat_ != null & lat != null) {
			lat.setText(lat_.toString());
		}
		if (lon_ != null & lon != null) {
			lon.setText(lon_.toString());
		}
	}

	private class MapPanel extends JPanel {
		private static final long serialVersionUID = 4153098045458832521L;
		
		/* Image Dimensons */
		private static final int WIDTH = 360;
		private static final int HEIGHT = 240;
		private BufferedImage map;
		
		public MapPanel() {
			this.setPreferredSize(new Dimension(WIDTH, HEIGHT));
			
			// Load map image as background
			try {
				map = ImageIO.read(new File("images/course_map.png"));
			} catch (Exception e) {
				System.out.println("Unable to open map background!");
			}
		}
	
		@Override
		public void paintComponent(Graphics g) {
			g.drawImage(map, 0, 0, WIDTH, HEIGHT, Color.black, null);			
		}
	
		/*private void setPixels() {
			pixelX = (int)(WIDTH * (currLoc.getX() - UL.getX()) / lon_width);
			pixelY = (int)(HEIGHT * (UL.getY() - currLoc.getY()) / lat_height);	
		}*/
	}
}