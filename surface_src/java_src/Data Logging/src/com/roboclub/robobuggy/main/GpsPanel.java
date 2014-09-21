package com.roboclub.robobuggy.main;

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.Date;

import javax.imageio.ImageIO;
import java.awt.Graphics;
import java.awt.GridLayout;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.map.Point;
import com.roboclub.robobuggy.map.Rect;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class GpsPanel extends SerialPanel {
	private static final long serialVersionUID = 1399590586061060311L;
	
	/* Constants for Serial Communication */
	/** Header char array for picking correct serial port */
	private static final char[] HEADER = {'$', 'G','P','G','G','A'};
	/** Length of the header char array */
	private static final int HEADER_LEN = 6;
	/** Baud rate for serial port */
	private static final int BAUDRATE = 9600;
	/** Index of latitude data as received during serial communication */
	private static final int LAT_NUM = 1;
	/** Index of latitude direction as received during serial communication */
	private static final int LAT_DIR = 2;
	/** Index of longitude data as received during serial communication */
	private static final int LONG_NUM = 3;
	/** Index of longitude direction as received during serial communication */
	private static final int LONG_DIR = 4;
	/** Index to log updated GPS position */
	private static final int LOG = 5;
	
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
		this.setLayout(new GridLayout(2, 1));
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
	
	private double parseLat(String latNum) {
		return (Double.valueOf(latNum.substring(0,2)) + 
				(Double.valueOf(latNum.substring(2)) / 60.0));
	}
	
	private double parseLon(String lonNum) {
		return (Double.valueOf(lonNum.substring(0,3)) + 
				(Double.valueOf(lonNum.substring(3)) / 60.0));
	}
	
	private void logData() {
		RobotLogger rl = RobotLogger.getInstance();
	    Date now = new Date();
	    long time_in_millis = now.getTime();
	    rl.sensor.logGps(time_in_millis, currLoc.getX(), currLoc.getY()); //TODO change names X = lat, Y = lon 
	}
	
	/**
	 * GpsListener is an event handler for incoming serial messages. It
	 * is notified every time a complete serial message is received by
	 * the serial port for the given GpsPanel.
	 */
	private class GpsListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			if(Gui.getInstance().GetPlayPauseState()) {
				char[] tmp = event.getBuffer();
				int length = event.getLength();
				int index = 0;
				
				if (tmp != null && event.getLength() > HEADER_LEN) {
					String curVal = "";
					
					// Filter messages by header
					for (int i = 0; i < HEADER_LEN; i++) {
						if (tmp[i] != HEADER[i]) return;
					}
					
					// Parse message data
					for (int i = HEADER_LEN+1; i < length; i++ ) {
						if (tmp[i] == ',' || tmp[i] == '\n') {
							try {
								switch ( index ) {
								case LAT_NUM:
									currLoc.setY( parseLat(curVal) );
									break;
								case LAT_DIR:
									if (curVal.equalsIgnoreCase("S")) currLoc.setY( -1 * currLoc.getY());
									break;
								case LONG_NUM:
									currLoc.setX( parseLon(curVal) );
									break;
								case LONG_DIR:
									if (curVal.equalsIgnoreCase("W")) currLoc.setX( -1 * currLoc.getX());
									break;
								case LOG:
									logData();
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
				}
			}
		}
	}
}