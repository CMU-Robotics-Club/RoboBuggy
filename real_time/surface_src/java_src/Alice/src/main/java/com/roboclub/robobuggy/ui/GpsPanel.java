package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import org.openstreetmap.gui.jmapviewer.*;
import org.openstreetmap.gui.jmapviewer.interfaces.ICoordinate;
import org.openstreetmap.gui.jmapviewer.interfaces.MapMarker;
import org.openstreetmap.gui.jmapviewer.interfaces.MapPolygon;
import org.openstreetmap.gui.jmapviewer.interfaces.TileCache;
import org.openstreetmap.gui.jmapviewer.tilesources.BingAerialTileSource;

import javax.imageio.ImageIO;
import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.List;


/**
 * panel for dynamic map
 */
public class GpsPanel extends JPanel {
	
	private static final long serialVersionUID = 42L;

	private static final int WAYPOINT_EDITOR_PANEL_WIDTH = 200;
	private static final int WAYPOINT_EDITOR_PANEL_HEIGHT = 100;
	private static final int WAYPOINT_SAVER_PANEL_HEIGHT = 50;
	
	private double theta = 0;
	private MapPolygonImpl directionLine;
	
	private JMapViewerTree mapTree;
	
	private double mapViewerLat = 40.440138;
	private double mapViewerLon = -79.945306;
	private String mapCacheFolderDiskPath = "images/cachedCourseMap";
	
	private double mapDragX = -1;
	private double mapDragY = -1;

	@SuppressWarnings("unused") //this subscriber is used to generate callbacks
	private Subscriber gpsSub;
	
	/**
	 * Construct a new {@link GpsPanel}
	 */
	public GpsPanel(){

		directionLine = new MapPolygonImpl(
				new Coordinate(0, 0),
				new Coordinate(0, 0),
				new Coordinate(0, 1)
		);

		initMapTree();
		addCacheToTree();
		
		this.add(mapTree);
		gpsSub = new Subscriber(NodeChannel.GPS.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {

				double latitude = ((GpsMeasurement)m).getLatitude();
				double longitude = ((GpsMeasurement) m).getLongitude();

				if(((GpsMeasurement)m).getWest()) {
					longitude = -longitude;
				}

				GpsPanel.this.addPointsToMapTree(Color.BLUE, new LocTuple(latitude, longitude));
				GpsPanel.this.repaint();  // refresh screen

			}
		});

		mapTree.getViewer().addMapPolygon(directionLine);



		// testing code
		addPointsToMapTree(Color.BLUE, new LocTuple(mapViewerLat, mapViewerLon));
		addPointsToMapTree(Color.BLUE, new LocTuple(mapViewerLat + 0.001, mapViewerLon));
	}

	private void updateArrow() {
		List<MapPolygon> polygons = mapTree.getViewer().getMapPolygonList();
			List<MapMarker> markers = mapTree.getViewer().getMapMarkerList();
			if (markers.size() >= 2) {
				MapMarker backMarker = markers.get(markers.size() - 2);
				MapMarker frontMarker = markers.get(markers.size() - 1);

				double endpointLat = 50 * (frontMarker.getLat() - backMarker.getLat()) + frontMarker.getLat();
				double endpointLon = 50 * (frontMarker.getLon() - backMarker.getLon()) + frontMarker.getLon();

				polygons.clear();
				addLineToMap(new LocTuple(frontMarker.getLat(), frontMarker.getLon()), new LocTuple(endpointLat, endpointLon), Color.RED);
			}
	}

	private void addCacheToTree() {
		try {
			TileCache courseCache = new MemoryTileCache();
			File mapCacheDir = new File(mapCacheFolderDiskPath);
			if(!mapCacheDir.isDirectory() || !mapCacheDir.exists()) {
				throw new IOException("cache dir isn't properly structured or doesn't exist");
			}
			
			FilenameFilter filter = (dir, name) -> {
                // TODO Auto-generated method stub
				return name.contains("png");
			};
			String[] cachedImages = mapCacheDir.list(filter);
			if (cachedImages == null) {
				return;
			}
			for(String imageName : cachedImages) {
				BufferedImage tileImageSource = ImageIO.read(new File(mapCacheDir.getAbsolutePath() + "/" + imageName));
				String[] tileCoords = imageName.substring(0, imageName.indexOf(".")).split("_");
				int xCoord = Integer.parseInt(tileCoords[0]);
				int yCoord = Integer.parseInt(tileCoords[1]);
				int zoomLevel = Integer.parseInt(tileCoords[2]);
				
				Tile cacheInsert = new Tile(mapTree.getViewer().getTileController().getTileSource(),
						xCoord, yCoord, zoomLevel, tileImageSource);
				cacheInsert.setLoaded(true);
				courseCache.addTile(cacheInsert);
			}
			
			mapTree.getViewer().getTileController().setTileCache(courseCache);
		}
		catch (IOException e) {
			new RobobuggyLogicNotification("Something is wrong with the map cache: " + e.getMessage(), RobobuggyMessageLevel.EXCEPTION);
		}
	}

	/**
	 * @param points points to add to the map
	 * @param thisColor color of the point
	 */
	public void addPointsToMapTree(Color thisColor,LocTuple...points) {
		for (LocTuple point : points) {
			mapTree.getViewer().addMapMarker(new MapMarkerDot(thisColor, point.getLatitude(), point.getLongitude()));
		}
	}

	/**
	 * @param point1 1st endpoint of line to add
	 * @param point2 2nd endpoint of line to add
	 * @param lineColor color of the line
	 */
	public void addLineToMap(LocTuple point1, LocTuple point2, Color lineColor) {
		MapPolygonImpl polygon = new MapPolygonImpl(
				new Coordinate(point1.getLatitude(), point1.getLongitude()),
				new Coordinate(point1.getLatitude(), point1.getLongitude()),
				new Coordinate(point2.getLatitude(), point2.getLongitude())
		);
		polygon.setColor(lineColor);
		mapTree.getViewer().addMapPolygon(polygon);
	}

	/**
	 * @param originPoint the origin point of the ray
	 * @param angle the heading of the ray
	 * @param lineColor the color of the line
	 */
	public void addLineToMap(LocTuple originPoint, double angle, Color lineColor) {
		angle = -angle;
		double scalingFactor = 0.00005;
		double dx = Math.cos(angle) * scalingFactor;
		double dy = Math.sin(angle) * scalingFactor;

		LocTuple endpoint = new LocTuple(originPoint.getLatitude() + dx, originPoint.getLongitude() + dy);
		addLineToMap(originPoint, endpoint, lineColor);
	}

	private void initMapTree() {
		mapTree = new JMapViewerTree("Buggy");
		mapTree.getViewer().setTileSource(new BingAerialTileSource());
		mapTree.setSize(getWidth(), getHeight());
		mapTree.getViewer().setSize(getWidth(), getHeight());
		mapTree.getViewer().setTileLoader(new OsmTileLoader(mapTree.getViewer()));
		mapTree.getViewer().setDisplayPosition(new Coordinate(mapViewerLat, mapViewerLon), 17);
		mapTree.getViewer().addMouseListener(new MouseListener() {
			
			@Override
			public void mouseReleased(MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void mousePressed(MouseEvent e) {
				// TODO Auto-generated method stub
				mapDragX = e.getX();
				mapDragY = e.getY();

				ICoordinate coordinate = mapTree.getViewer().getPosition((int) mapDragX, (int) mapDragY);

//				List<MapMarker> markers = mapTree.getViewer().getMapMarkerList();
//				double toleranceDist = 0.0002;
//				for (int i = 0; i < markers.size(); i++) {
//					MapMarker marker = markers.get(i);
//					double dlat = marker.getLat() - coordinate.getLat();
//					double dlon = marker.getLon() - coordinate.getLon();
//
//					double dist = Math.sqrt(dlat * dlat + dlon * dlon);
//					if (dist <= toleranceDist) {
//						mapTree.getViewer().setDisplayPosition(marker.getCoordinate(), mapTree.getViewer().getZoom());
//						waypointList.setSelectedIndex(i);
//						return;
//					}
//				}
			}
			
			@Override
			public void mouseExited(MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void mouseEntered(MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void mouseClicked(MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
		});
		mapTree.getViewer().addMouseMotionListener(new MouseMotionListener() {
			@Override
			public void mouseMoved(MouseEvent e) {
				// TODO Auto-generated method stub

			}

			@Override
			public void mouseDragged(MouseEvent e) {
				// TODO Auto-generated method stub

				int zoomLevel = mapTree.getViewer().getZoom();

				mapViewerLat -= ((mapDragY - e.getY()) * 0.001) / (zoomLevel * 1000);
				mapViewerLon -= ((e.getX() - mapDragX) * 0.001) / (zoomLevel * 1000);
				mapTree.getViewer().setDisplayPosition(new Coordinate(mapViewerLat, mapViewerLon), zoomLevel);
			}
		});
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

		mapTree.setBounds(0, 0, getWidth(), getHeight());
		mapTree.getViewer().setSize(getWidth(), getHeight());


	}
}
