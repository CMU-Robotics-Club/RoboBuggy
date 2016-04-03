package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import org.openstreetmap.gui.jmapviewer.Coordinate;
import org.openstreetmap.gui.jmapviewer.MapMarkerDot;
import org.openstreetmap.gui.jmapviewer.MapPolygonImpl;

import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Graphics;


/**
 * panel for dynamic map
 */
public class GpsPanel extends JPanel {
	
	private static final long serialVersionUID = 42L;
	
	private double theta = 0;
	private MapPolygonImpl directionLine;
	
	private Map map;
	private MapMarkerDot destinationPoint;

	/**
	 * @return the destinationPoint
	 */
	public synchronized MapMarkerDot getDestinationPoint() {
		return destinationPoint;
	}


	/**
	 * @param destinationPoint the destinationPoint to set
	 */
	public synchronized void setDestinationPoint(MapMarkerDot destinationPoint) {
		this.destinationPoint = destinationPoint;
	}


	private GPSPoseMessage lastpose;

	@SuppressWarnings("unused") //this subscriber is used to generate callbacks
	private Subscriber gpsSub;
	
	/**
	 * Construct a new {@link GpsPanel}
	 */
	public GpsPanel(){

		map = new Map();
		map.setBounds(0, 0, getWidth(), getHeight());
		destinationPoint = new MapMarkerDot(0.0, 0.0);

		map.getMapTree().getViewer().addMapMarker(destinationPoint);

		directionLine = new MapPolygonImpl(
				new Coordinate(0, 0),
				new Coordinate(0, 0),
				new Coordinate(0, 1)
		);

		gpsSub = new Subscriber(NodeChannel.GPS.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {

				double latitude = ((GpsMeasurement) m).getLatitude();
				double longitude = ((GpsMeasurement) m).getLongitude();

				map.addPointsToMapTree(Color.BLUE, new LocTuple(latitude, longitude));
				map.updateArrow();

				map.repaint();
				GpsPanel.this.repaint();  // refresh screen

			}
		});
		
		new Subscriber(NodeChannel.POSE.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				// TODO Auto-generated method stub
				
				GPSPoseMessage curpose = (GPSPoseMessage) m;
				map.addPointsToMapTree(Color.RED, new LocTuple(curpose.getLatitude(), curpose.getLongitude()));
				map.addLineToMap(new LocTuple(curpose.getLatitude(), curpose.getLongitude()),
									Math.toRadians(curpose.getHeading()), Color.CYAN, true);
			}
		});

		this.add(map);
		map.repaint();

	}


	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		map.setBounds(0, 0, getWidth(), getHeight());
	}
}
