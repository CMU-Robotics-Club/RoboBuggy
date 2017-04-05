package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
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
    private int zoomLevel = 20;

    private double mapDragX = -1;
    private double mapDragY = -1;
    private static final int MAX_POINT_BUF_SIZE = 3000;

    private MapMarkerDot currentWaypoint = new MapMarkerDot(0, 0);
    private MapPolygonImpl currentSteeringCommandMapObj = new MapPolygonImpl();
    private MapPolygonImpl currentHeadingMapObj = new MapPolygonImpl();
    private MapPolygonImpl desiredHeadingMapObj = new MapPolygonImpl();

    /**
     * initializes a new Map with cache loaded
     */
    public Map() {
        initMapTree();
        addCacheToTree();
        this.add(getMapTree());

        currentWaypoint.setColor(Color.BLUE);
        getMapTree().getViewer().addMapMarker(currentWaypoint);
        getMapTree().getViewer().addMapPolygon(currentSteeringCommandMapObj);
        getMapTree().getViewer().addMapPolygon(currentHeadingMapObj);
        getMapTree().getViewer().addMapPolygon(desiredHeadingMapObj);

        //adds track buggy  
        new Subscriber("Map", NodeChannel.POSE.getMsgPath(), new MessageListener() {
            //TODO make this optional
            @Override
            public void actionPerformed(String topicName, Message m) {
                GPSPoseMessage gpsM = (GPSPoseMessage) m;
                zoomLevel = getMapTree().getViewer().getZoom();
                getMapTree().getViewer().setDisplayPosition(new Coordinate(gpsM.getLatitude(),
                        gpsM.getLongitude()), zoomLevel);
                addPointsToMapTree(Color.RED, new LocTuple(gpsM.getLatitude(), gpsM.getLongitude()));

                getMapTree().getViewer().removeMapMarker(currentWaypoint);
//                currentWaypoint.setLat(WayPointFollowerPlanner.currentWaypoint.getLatitude());
//                currentWaypoint.setLon(WayPointFollowerPlanner.currentWaypoint.getLongitude());
                getMapTree().getViewer().addMapMarker(currentWaypoint);

                getMapTree().getViewer().removeMapPolygon(desiredHeadingMapObj);
                desiredHeadingMapObj = new MapPolygonImpl(
                        new Coordinate(gpsM.getLatitude(), gpsM.getLongitude()),
                        new Coordinate(gpsM.getLatitude() + 0.0001 * Math.sin(WayPointFollowerPlanner.currentDesiredHeading), gpsM.getLongitude() + 0.0001 *
                                Math.cos(WayPointFollowerPlanner.currentDesiredHeading)),
                        new Coordinate(gpsM.getLatitude(), gpsM.getLongitude())
                );
                desiredHeadingMapObj.setColor(Color.GREEN);
                getMapTree().getViewer().addMapPolygon(desiredHeadingMapObj);

                double currentHeading = gpsM.getCurrentState().get(3, 0);
                getMapTree().getViewer().removeMapPolygon(currentHeadingMapObj);
                currentHeadingMapObj = new MapPolygonImpl(
                        new Coordinate(gpsM.getLatitude(), gpsM.getLongitude()),
                        new Coordinate(gpsM.getLatitude() + 0.0001 * Math.sin(currentHeading), gpsM.getLongitude() + 0.0001 * Math.cos(currentHeading)),
                        new Coordinate(gpsM.getLatitude(), gpsM.getLongitude())
                );
                currentHeadingMapObj.setColor(Color.RED);
                getMapTree().getViewer().addMapPolygon(currentHeadingMapObj);

                getMapTree().getViewer().removeMapPolygon(currentSteeringCommandMapObj);
                currentSteeringCommandMapObj = new MapPolygonImpl(
                        new Coordinate(gpsM.getLatitude(), gpsM.getLongitude()),
                        new Coordinate(gpsM.getLatitude() + 0.0001 * Math.sin(WayPointFollowerPlanner
                                .currentCommandedAngle + currentHeading), gpsM.getLongitude() + 0.0001 * Math.cos(WayPointFollowerPlanner
                                .currentCommandedAngle + currentHeading)),
                        new Coordinate(gpsM.getLatitude(), gpsM.getLongitude())
                );
                currentSteeringCommandMapObj.setColor(Color.BLUE);
                getMapTree().getViewer().addMapPolygon(currentSteeringCommandMapObj);

            }
        });

        new Subscriber("map", NodeChannel.GPS.getMsgPath(), ((topicName, m) -> {
            GpsMeasurement gps = ((GpsMeasurement) m);
            addPointsToMapTree(Color.BLACK, new LocTuple(gps.getLatitude(), gps.getLongitude()));
        }));
        new Subscriber("map", NodeChannel.DRIVE_CTRL.getMsgPath(), (topicName, m) -> {
            DriveControlMessage dcm = ((DriveControlMessage) m);
            currentWaypoint.setLat(dcm.getWaypoint().getLatitude());
            currentWaypoint.setLon(dcm.getWaypoint().getLongitude());
        });

    }


    private void initMapTree() {
        setMapTree(new JMapViewerTree("Buggy"));
        getMapTree().getViewer().setTileSource(new BingAerialTileSource());
        getMapTree().setSize(getWidth(), getHeight());
        getMapTree().getViewer().setSize(getWidth(), getHeight());
        getMapTree().getViewer().setTileLoader(new OsmTileLoader(getMapTree().getViewer()));
        getMapTree().getViewer().setDisplayPosition(new Coordinate(mapViewerLat, mapViewerLon), zoomLevel);
        getMapTree().getViewer().addMouseListener(new MouseListener() {

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
        getMapTree().getViewer().addMouseMotionListener(new MouseMotionListener() {
            @Override
            public void mouseMoved(MouseEvent e) {
                // TODO Auto-generated method stub

            }

            @Override
            public void mouseDragged(MouseEvent e) {
                // TODO Auto-generated method stub

                zoomLevel = getMapTree().getViewer().getZoom();

                mapViewerLat -= ((mapDragY - e.getY()) * 0.001) / (zoomLevel * 1000);
                mapViewerLon -= ((e.getX() - mapDragX) * 0.001) / (zoomLevel * 1000);
                getMapTree().getViewer().setDisplayPosition(new Coordinate(mapViewerLat, mapViewerLon), zoomLevel);
            }
        });
    }

    private void addCacheToTree() {
        try {
            TileCache courseCache = new MemoryTileCache();
            String mapCacheFolderDiskPath = "images/cachedCourseMap";
            File mapCacheDir = new File(mapCacheFolderDiskPath);
            if (!mapCacheDir.isDirectory() || !mapCacheDir.exists()) {
                throw new IOException("cache dir isn't properly structured or doesn't exist");
            }

            FilenameFilter filter = (dir, name) -> {
                // TODO Auto-generated method stub
                if (name.contains("png")) {
                    return true;
                }
                return false;
            };
            String[] cachedImages = mapCacheDir.list(filter);
            if (cachedImages == null) {
                return;
            }
            for (String imageName : cachedImages) {
                BufferedImage tileImageSource = ImageIO.read(new File(mapCacheDir.getAbsolutePath() + "/" + imageName));
                String[] tileCoords = imageName.substring(0, imageName.indexOf(".")).split("_");
                int xCoord = Integer.parseInt(tileCoords[0]);
                int yCoord = Integer.parseInt(tileCoords[1]);
                zoomLevel = Integer.parseInt(tileCoords[2]);

                Tile cacheInsert = new Tile(getMapTree().getViewer().getTileController().getTileSource(),
                        xCoord, yCoord, zoomLevel, tileImageSource);
                cacheInsert.setLoaded(true);
                courseCache.addTile(cacheInsert);
            }

            getMapTree().getViewer().getTileController().setTileCache(courseCache);
        } catch (IOException e) {
            new RobobuggyLogicNotification("Something is wrong with the map cache: " + e.getMessage(), RobobuggyMessageLevel.EXCEPTION);
        }
    }

    /**
     * updates the current arrow displaying on the GUI - shows orientation based on GPS
     */
    public void updateArrow() {
        List<MapPolygon> polygons = getMapTree().getViewer().getMapPolygonList();
        List<MapMarker> markers = getMapTree().getViewer().getMapMarkerList();
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
     * @param points    points to add to the map
     * @param thisColor color of the point
     */
    public void addPointsToMapTree(Color thisColor, LocTuple... points) {
        List<MapMarker> markers = getMapTree().getViewer().getMapMarkerList();
        while (markers.size() > MAX_POINT_BUF_SIZE - points.length) {
            markers.remove(0);
        }
        for (LocTuple point : points) {
            getMapTree().getViewer().addMapMarker(new MapMarkerDot(thisColor, point.getLatitude(), point.getLongitude()));
        }
    }

    /**
     * @param point1    1st endpoint of line to add
     * @param point2    2nd endpoint of line to add
     * @param lineColor color of the line
     */
    public void addLineToMap(LocTuple point1, LocTuple point2, Color lineColor) {
        MapPolygonImpl polygon = new MapPolygonImpl(
                new Coordinate(point1.getLatitude(), point1.getLongitude()),
                new Coordinate(point1.getLatitude(), point1.getLongitude()),
                new Coordinate(point2.getLatitude(), point2.getLongitude())
        );
        polygon.setColor(lineColor);
        getMapTree().getViewer().addMapPolygon(polygon);
    }

    /**
     * @param originPoint   the origin point of the ray
     * @param angle         the heading of the ray in radians
     * @param lineColor     the color of the line
     * @param clearPrevLine update the line or add a new one
     */
    public void addLineToMap(LocTuple originPoint, double angle, Color lineColor, boolean clearPrevLine) {
        if (clearPrevLine) {
            getMapTree().getViewer().getMapPolygonList().clear();
        }

        double scalingFactor = 0.0005;
        double dx = Math.cos(angle) * scalingFactor;
        double dy = Math.sin(angle) * scalingFactor;

        LocTuple endpoint = new LocTuple(originPoint.getLatitude() + dy, originPoint.getLongitude() + dx);
        addLineToMap(originPoint, endpoint, lineColor);
    }

    @Override
    public void paintComponent(Graphics g) {

        getMapTree().setBounds(0, 0, getWidth(), getHeight());
        getMapTree().getViewer().setSize(getWidth(), getHeight());

    }


    /**
     * @return the mapTree
     */
    public JMapViewerTree getMapTree() {
        return mapTree;
    }


    /**
     * @param mapTree the mapTree to set
     */
    public void setMapTree(JMapViewerTree mapTree) {
        this.mapTree = mapTree;
    }

}
