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
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * {@link JPanel} used to display GPS data
 */
public class GpsPanel extends JPanel {	
	private static final long serialVersionUID = 42L;
	private ArrayList<LocTuple> locs;
	private LocTuple imgNorthWest;
	private LocTuple imgSouthEast;
	private BufferedImage map;
	private boolean setup;
	private int frameWidth;
	private int frameHeight;
	
	private double theta = 0;
	
	@SuppressWarnings("unused") //this subscriber is used to generate callbacks 
	private Subscriber gpsSub;
	
	/**
	 * Construct a new {@link GpsPanel}
	 */
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
		
		gpsSub = new Subscriber(NodeChannel.GPS.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				double latitude = ((GpsMeasurement)m).getLatitude();
				double longitude = ((GpsMeasurement)m).getLongitude();

				//todo put mag based on dir
				if(((GpsMeasurement)m).getWest()) {
					longitude = -longitude;
				}
				
				locs.add(new LocTuple(latitude, longitude));
//				int gpsSize = locs.size(); // This is new: looks locs.size
//				if (gpsSize > 20) {        // if size > 20, remove first object
//					locs.remove(0);
//				}
			 // refresh screen
			    Gui.getInstance().fixPaint();
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
		
		
		int cDiameter = 5;
		g2d.setColor(Color.RED);
		g2d.fillOval((int)px, (int)py, cDiameter, cDiameter);
	}
	
	private void drawArrow(Graphics2D g2d, double x1, double y1, double theta, double size){
		double x2 = x1 + rotate(size, 0, theta)[0];
		double y2 = y1 + rotate(size, 0, theta)[1];
		g2d.drawLine((int)x1, (int)y1, (int)x2, (int)y2);
		double x3 = x2 + rotate(size/2, 0, theta + 3*Math.PI/4)[0];
		double y3 = y2 + rotate(size/2, 0, theta + 3*Math.PI/4)[1];
		g2d.drawLine((int)x2, (int)y2, (int)x3, (int)y3);
		double x4 = x2 + rotate(size/2, 0, theta - 3*Math.PI/4)[0];
		double y4 = y2 + rotate(size/2, 0, theta - 3*Math.PI/4)[1];
		g2d.drawLine((int)x2, (int)y2, (int)x4, (int)y4);
	}
	
	private double[] rotate(double x, double y, double theta){
		double xp = (x*Math.cos(theta) + y*Math.sin(theta));
		double yp = (-x*Math.sin(theta) + y*Math.cos(theta));
		return new double[] {xp, yp};
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
		g2d.setColor(Color.blue);
		drawArrow(g2d, 0.5*frameWidth, 0.5*frameHeight, theta, 0.1*frameWidth);
		theta += 0.01;
	
		for	(LocTuple mTuple : locs) {
			drawTuple(g2d, mTuple);
		}
		g2d.dispose();
	}
	
	//Ad hoc image processing code to follow
	public BufferedImage processImage(BufferedImage image, double theta){
		BufferedImage temp = new BufferedImage((int)(1.42*(image.getWidth()+image.getHeight())) + 2,(int)(1.42*(image.getWidth()+image.getHeight())) + 2, BufferedImage.TYPE_INT_ARGB);
		for(int x = 0; x < temp.getWidth(); x++){
			for(int y = 0; y < temp.getHeight(); y++){
				temp.setRGB(x, y, new Color(255, 255, 255, 0).getRGB());
			}
		}
		for(int x = 0; x < image.getWidth(); x++){
			for(int y = 0; y < image.getHeight(); y++){
				Color c = new Color(image.getRGB(x, y));
				c = new Color(c.getRed(), c.getGreen(), c.getBlue(), c.getAlpha());
				int xp = (int)(0.71*(image.getWidth() + image.getHeight())) + 1 + (int) (x*Math.cos(theta) + y*Math.sin(theta));
				int yp = (int)(0.71*(image.getWidth() + image.getHeight())) + 1 + (int) (-x*Math.sin(theta) + y*Math.cos(theta));
				try{
					//Overwrite anything within one pixel of this value to prevent rounding issues
					for(int xd = -1; xd <= 1; xd++){
						for(int yd = -1; yd <= 1; yd++){
							temp.setRGB(xp+xd, yp+yd, c.getRGB());
						}
					}
				}catch(Exception e)
				{
					System.out.println(x + " " + y + " " + xp + " " + yp);
					System.exit(0);
				}
			}
		}
		return temp;
	}
}