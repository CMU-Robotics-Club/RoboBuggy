package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

import org.openstreetmap.gui.jmapviewer.Coordinate;
import org.openstreetmap.gui.jmapviewer.JMapViewerTree;
import org.openstreetmap.gui.jmapviewer.MapMarkerDot;
import org.openstreetmap.gui.jmapviewer.MemoryTileCache;
import org.openstreetmap.gui.jmapviewer.OsmTileLoader;
import org.openstreetmap.gui.jmapviewer.Tile;
import org.openstreetmap.gui.jmapviewer.interfaces.TileCache;
import org.openstreetmap.gui.jmapviewer.tilesources.BingAerialTileSource;

import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.main.RobobuggyLogicException;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

public class GpsPanel extends JPanel {	
	
	private static final long serialVersionUID = 42L;
	
	private Subscriber gpsSub;
	private JMapViewerTree mapTree;
	
	private double mapViewerLat = 40.440138;
	private double mapViewerLon = -79.945306;
	private String mapCacheFolderDiskPath = "images/cachedCourseMap";
	
	
	public GpsPanel(){
		
		initMapTree();
		addCacheToTree();
		
		this.add(mapTree);
		
		gpsSub = new Subscriber(SensorChannel.GPS.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				double latitude = ((GpsMeasurement)m).latitude;
				double longitude = ((GpsMeasurement)m).longitude;

				if(((GpsMeasurement)m).west) {
					longitude = -longitude;
				}
				
				mapTree.getViewer().addMapMarker(new MapMarkerDot(Color.BLUE, latitude, longitude));
				GpsPanel.this.repaint();  // refresh screen
			}
		});		
		
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
				int xCoord = Integer.valueOf(tileCoords[0]);
				int yCoord = Integer.valueOf(tileCoords[1]);
				int zoomLevel = Integer.valueOf(tileCoords[2]);
				
				Tile cacheInsert = new Tile(mapTree.getViewer().getTileController().getTileSource(), xCoord, yCoord, zoomLevel, tileImageSource);
				cacheInsert.setLoaded(true);
				courseCache.addTile(cacheInsert);
			}
			
			mapTree.getViewer().getTileController().setTileCache(courseCache);
		}
		catch (IOException e) {
			new RobobuggyLogicException("Something is wrong with the map cache: " + e.getMessage(), MessageLevel.EXCEPTION);
		}
	}

	private void initMapTree() {
		mapTree = new JMapViewerTree("Buggy");
		mapTree.getViewer().setTileSource(new BingAerialTileSource());
		mapTree.setSize(getWidth(), getHeight());
		mapTree.getViewer().setSize(getWidth(), getHeight());
		mapTree.getViewer().setTileLoader(new OsmTileLoader(mapTree.getViewer()));
		mapTree.getViewer().setDisplayPosition(new Coordinate(mapViewerLat, mapViewerLon), 17);
		mapTree.getViewer().addMouseMotionListener(new MouseMotionListener() {
			
			private int prevEx = -1;
			private int prevEy = -1;
	
			@Override
			public void mouseMoved(MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void mouseDragged(MouseEvent e) {
				// TODO Auto-generated method stub
				if(prevEx == -1 || prevEy == -1) {
					prevEx = e.getX();
					prevEy = e.getY();
				}
				
				int zoomLevel = mapTree.getViewer().getZoom();
				
				mapViewerLat += ((prevEy - e.getY()) * 0.00001) / (zoomLevel * 10);
				mapViewerLon += ((e.getX() - prevEx) * 0.00001) / (zoomLevel * 10);
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