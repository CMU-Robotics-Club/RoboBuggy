package com.roboclub.robobuggy.main;

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.Date;

import javax.imageio.ImageIO;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

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
		super("GPS", BAUDRATE, HEADER, HEADER_LEN);
		
		if (!this.isConnected()) return;
		
		super.addListener(new GpsListener());
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
	
	private float parseLat(String latNum) {
		return (Float.valueOf(latNum.substring(0,2)) + 
				(Float.valueOf(latNum.substring(2)) / 60.0f));
	}
	
	private float parseLon(String lonNum) {
		return (Float.valueOf(lonNum.substring(0,3)) + 
				(Float.valueOf(lonNum.substring(3)) / 60.0f));
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
			if(Gui.GetGraphState() || Gui.GetPlayPauseState()) {
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
									setPixels();
									
									if (Gui.GetPlayPauseState()) logData();
									if (Gui.GetGraphState()) Gui.UpdateRobotPos(currLoc.getY(), currLoc.getX());
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