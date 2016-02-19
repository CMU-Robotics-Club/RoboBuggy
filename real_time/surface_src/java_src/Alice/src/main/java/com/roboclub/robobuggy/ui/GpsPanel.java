package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

import javax.imageio.ImageIO;
import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;

import org.openstreetmap.gui.jmapviewer.*;
import org.openstreetmap.gui.jmapviewer.interfaces.TileCache;
import org.openstreetmap.gui.jmapviewer.tilesources.BingAerialTileSource;


/**
 * panel for dynamic map
 */
public class GpsPanel extends JPanel {
	
	private static final long serialVersionUID = 42L;
	
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
				
				mapTree.getViewer().addMapMarker(new MapMarkerDot(Color.BLUE, latitude, longitude));
				GpsPanel.this.repaint();  // refresh screen

			}
		});

		mapTree.getViewer().addMapPolygon(new MapPolygonImpl(new Coordinate(mapViewerLat, mapViewerLon),
 													new Coordinate(mapViewerLat + 0.0001, mapViewerLon),
 													new Coordinate(mapViewerLat, mapViewerLon)));


	}
	
	private void addCacheToTree() {
		try {
			TileCache courseCache = new MemoryTileCache();
			File mapCacheDir = new File(mapCacheFolderDiskPath);
			if(!mapCacheDir.isDirectory() || !mapCacheDir.exists()) {
				throw new IOException("cache dir isn't properly structured or doesn't exist");
			}
			
			FilenameFilter filter = new FilenameFilter() {
				
				@Override
				public boolean accept(File dir, String name) {
					// TODO Auto-generated method stub
					if(name.contains("png")) {
						return true;
					}
					return false;
				}
			};
			String[] cachedImages = mapCacheDir.list(filter);
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

	public void addPointsToMapTree(LocTuple...points) {
		for (LocTuple point : points) {
			mapTree.getViewer().addMapMarker(new MapMarkerDot(Color.BLUE, point.getLatitude(), point.getLongitude()));
		}
	}

	public void addLineToMap(LocTuple point1, LocTuple point2) {
		mapTree.getViewer().addMapPolygon(new MapPolygonImpl(
				new Coordinate(point1.getLatitude(), point1.getLongitude()),
				new Coordinate(point1.getLatitude(), point1.getLongitude()),
				new Coordinate(point2.getLatitude(), point2.getLongitude())
		));
	}

	public void addLineToMap(LocTuple originPoint, double angle) {
		double scalingFactor = 0.001;
		double dx = originPoint.getLatitude() + Math.cos(angle) * scalingFactor;
		double dy = originPoint.getLongitude() + Math.sin(angle) * scalingFactor;

		LocTuple endpoint = new LocTuple(originPoint.getLatitude() + dx, originPoint.getLongitude() + dy);
		addLineToMap(originPoint, endpoint);
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
	

	@Override
	public void paintComponent(Graphics g) {

		mapTree.setBounds(0, 0, getWidth(), getHeight());
		mapTree.getViewer().setSize(getWidth(), getHeight());
		
	}
}