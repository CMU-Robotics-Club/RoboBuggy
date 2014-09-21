package com.roboclub.robobuggy.main;

import java.awt.image.BufferedImage;
import java.io.File;

import javax.imageio.ImageIO;

import java.awt.FlowLayout;
import java.awt.Graphics;

import com.roboclub.robobuggy.map.Point;
import com.roboclub.robobuggy.map.Rect;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class GpsPanel extends SerialPanel {
	private static final long serialVersionUID = 1399590586061060311L;
	
	/* Constants for Serial Communication */
	/** Header char array for picking correct serial port */
	private static final char[] HEADER = {'T', 'E', 'S', 'T'};
	/** Length of the header char array */
	private static final int HEADER_LEN = 4;
	/** Baud rate for serial port */
	private static final int BAUDRATE = 9600;
	/** Index of latitude data as received during serial communication */
	private static final int LATITUDE = 0;
	/** Index of longitude data as received during serial communication */
	private static final int LONGITUDE = 1;
	
	/* Panel Dimensons */
	private static final int PANEL_WIDTH = 400;
	private static final int PANEL_HEIGHT = 300;
	private static final int WIDTH = 400;
	private static final int HEIGHT = 270;
	
	private static final Rect MAP_COORD = new Rect(
			new Point(-79.94010, 40.442395),
			new Point(-79.948686, 40.442395),
			new Point(-79.94010, 40.438363),
			new Point(-79.948686, 40.438363));
	private Point currLoc;
	private BufferedImage map;
	
	/**
	 * Constructor for GPS panel on user interface. Initializes a serial communication
	 * for reading from serial port. For the ui, it creates a graph marking the last
	 * longitude and latitude read.
	 */
	public GpsPanel() {
		super("GPS", BAUDRATE, HEADER, HEADER_LEN);
		super.addListener(new GpsListener());
		this.setLayout(new FlowLayout(FlowLayout.LEFT, 1, 0));
		this.currLoc = new Point(0,0);
		
		// Load map image as background
		try {
			map = ImageIO.read(new File("images/course_map.png"));
		} catch (Exception e) {
			// TODO error handling
			return;
		}
	}
	
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		
		g.drawImage(map, 0, (PANEL_HEIGHT-HEIGHT), null);
		
		if (MAP_COORD.within(currLoc)) {
			//g.drawOval();
		}
	}
	
	/**
	 * GpsListener is an event handler for incoming serial messages. It
	 * is notified every time a complete serial message is received by
	 * the serial port for the given GpsPanel.
	 */
	private class GpsListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			char[] tmp = event.getBuffer();
			int index = 0;
			
			if (tmp != null && event.getLength() > HEADER_LEN) {
				String curVal = "";
				for (int i = HEADER_LEN; i < event.getLength(); i++ ) {
					if (tmp[i] == ',' || tmp[i] == '\n') {
						try {
							switch ( index ) {
							case LONGITUDE:
								currLoc.setX(Float.valueOf(curVal));
								break;
							case LATITUDE:
								currLoc.setY(Float.valueOf(curVal));
								break;
							default:
								return;
							}
							
							curVal = "";
							index++;
						} catch (Exception e) {
							System.out.println("Failed to parse gps message");
							return;
						}
						
					} else {
						curVal += tmp[i];
					}
				}

				//TODO redraw now
			}
		}
	}
}