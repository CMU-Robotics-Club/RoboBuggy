package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import org.openstreetmap.gui.jmapviewer.Coordinate;
import org.openstreetmap.gui.jmapviewer.JMapViewerTree;
import org.openstreetmap.gui.jmapviewer.MapMarkerDot;
import org.openstreetmap.gui.jmapviewer.MapPolygonImpl;
import org.openstreetmap.gui.jmapviewer.MemoryTileCache;
import org.openstreetmap.gui.jmapviewer.OsmTileLoader;
import org.openstreetmap.gui.jmapviewer.Tile;
import org.openstreetmap.gui.jmapviewer.interfaces.MapMarker;
import org.openstreetmap.gui.jmapviewer.interfaces.MapPolygon;
import org.openstreetmap.gui.jmapviewer.interfaces.TileCache;
import org.openstreetmap.gui.jmapviewer.tilesources.BingAerialTileSource;

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
import java.util.List;

/**
 * the map, it plots points where we are based on gps
 */
public class Map extends JPanel {

    private JMapViewerTree mapTree;

    private double mapViewerLat = 40.440138;
    private double mapViewerLon = -79.945306;
    private String mapCacheFolderDiskPath = "images/cachedCourseMap";

    private double mapDragX = -1;
    private double mapDragY = -1;

    /**
     * initializes a new Map with cache loaded
     */
    public Map() {
        initMapTree();
        addCacheToTree();
        this.add(mapTree);
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

    private void addCacheToTree() {
        try {
            TileCache courseCache = new MemoryTileCache();
            File mapCacheDir = new File(mapCacheFolderDiskPath);
            if(!mapCacheDir.isDirectory() || !mapCacheDir.exists()) {
                throw new IOException("cache dir isn't properly structured or doesn't exist");
            }

            FilenameFilter filter = (dir, name) -> {
                // TODO Auto-generated method stub
                if(name.contains("png")) {
                    return true;
                }
                return false;
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
     * updates the current arrow displaying on the GUI - shows orientation based on GPS
     */
    public void updateArrow() {
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

	/**
	 * @param points points to add to the map
	 * @param thisColor color of the point
	 */
	public void addPointsToMapTree(Color thisColor, LocTuple...points) {
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
		mapTree.getViewer().getMapPolygonList().clear();

		double scalingFactor = 0.05;
		double dx = Math.cos(angle) * scalingFactor;
		double dy = Math.sin(angle) * scalingFactor;

		LocTuple endpoint = new LocTuple(originPoint.getLatitude() + dx, originPoint.getLongitude() + dy);
		addLineToMap(originPoint, endpoint, lineColor);
	}

    @Override
    public void paintComponent(Graphics g) {

        mapTree.setBounds(0, 0, getWidth(), getHeight());
        mapTree.getViewer().setSize(getWidth(), getHeight());

    }

}
