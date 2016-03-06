package com.roboclub.robobuggy.ui;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.GpsMeasurement;
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
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.PrintStream;
import java.lang.reflect.Modifier;
import java.util.Date;
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

	private JScrollPane waypointListContainer;
	private WaypointList waypointList;
	private WaypointEditor waypointEditor;
	private WaypointSaver waypointSaver;

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

		waypointList = new WaypointList();

		waypointListContainer = new JScrollPane(waypointList);
		waypointListContainer.setBounds(0, 0, 0, 0);
		waypointListContainer.setBackground(Color.RED);
		waypointListContainer.setVisible(true);

		waypointEditor = new WaypointEditor(WAYPOINT_EDITOR_PANEL_WIDTH, WAYPOINT_EDITOR_PANEL_HEIGHT);
		waypointEditor.setVisible(true);
		waypointEditor.setBackground(Color.GRAY);

		waypointSaver = new WaypointSaver(WAYPOINT_SAVER_PANEL_HEIGHT);

		this.add(waypointEditor, 0);
		this.add(waypointListContainer, 0);
		this.add(waypointSaver);
		this.repaint();
		waypointListContainer.repaint();
		waypointList.repaint();
		waypointEditor.repaint();
		waypointSaver.repaint();


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
	 * @param points points to add to the map
	 * @param thisColor color of the point
	 */
	public void addPointsToMapTree(Color thisColor,LocTuple...points) {
		for (LocTuple point : points) {
			mapTree.getViewer().addMapMarker(new MapMarkerDot(thisColor, point.getLatitude(), point.getLongitude()));
			((DefaultListModel<String>) waypointList.getModel()).addElement(point.getLatitude() + ", " + point.getLongitude());
			waypointListContainer.repaint();
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

	/**
	 * Waypoint list that keeps track of the points currently on the map
	 */
	private class WaypointList extends JList<String> {

		WaypointList() {
			super(new DefaultListModel<>());
			setVisibleRowCount(-1);
			setSelectionMode(ListSelectionModel.SINGLE_INTERVAL_SELECTION);
			setLayoutOrientation(JList.VERTICAL);

			addListSelectionListener(new ListSelectionListener() {
				@Override
				public void valueChanged(ListSelectionEvent e) {
					int index = waypointList.getSelectedIndex();
					List<MapMarker> markers = mapTree.getViewer().getMapMarkerList();
					for (MapMarker marker : markers) {
						MapMarkerDot dot = (MapMarkerDot) marker;
						dot.setBackColor(Color.YELLOW);
					}
					MapMarkerDot dot = ((MapMarkerDot) markers.get(index));
					dot.setBackColor(Color.RED);
					waypointEditor.editPoint(index);

					mapTree.getViewer().repaint();
				}
			});
		}

	}

	/**
	 * Waypoint editor - allows us to modify waypoints on the fly
	 */
	private class WaypointEditor extends JPanel {

		private JTextField latField;
		private JTextField lonField;
		private int dotIndexToEdit;

		WaypointEditor(int width, int height) {
			super();

			setSize(width, height);

			latField = new JTextField();
			lonField = new JTextField();

			latField.setBounds(0, 0, this.getWidth(), this.getHeight()/2);
			lonField.setBounds(0, this.getHeight()/2, this.getWidth(), this.getHeight()/2);

			dotIndexToEdit = 0;

			latField.addActionListener(e -> {
				MapMarkerDot dotToEdit = (MapMarkerDot) mapTree.getViewer().getMapMarkerList().get(dotIndexToEdit);
				dotToEdit.setLat(Double.parseDouble(latField.getText()));
				mapTree.getViewer().repaint();
				((DefaultListModel<String>) waypointList.getModel()).setElementAt(
						dotToEdit.getLat() + ", " + dotToEdit.getLon(),
						dotIndexToEdit
				);
				waypointList.repaint();
			});
			lonField.addActionListener(e -> {
				MapMarkerDot dotToEdit = (MapMarkerDot) mapTree.getViewer().getMapMarkerList().get(dotIndexToEdit);
				dotToEdit.setLon(Double.parseDouble(lonField.getText()));
				((DefaultListModel<String>) waypointList.getModel()).setElementAt(
						dotToEdit.getLat() + ", " + dotToEdit.getLon(),
						dotIndexToEdit
				);
				mapTree.getViewer().repaint();
				waypointList.repaint();
			});

			this.add(latField);
			this.add(lonField);
		}

		public void editPoint(int dotIndex) {
			dotIndexToEdit = dotIndex;
			MapMarkerDot dot = (MapMarkerDot) mapTree.getViewer().getMapMarkerList().get(dotIndex);
			latField.setText(String.valueOf(dot.getLat()));
			lonField.setText(String.valueOf(dot.getLon()));
		}

		@Override
		public void paint(Graphics g) {
			super.paint(g);

			latField.setBounds(0, 0, this.getWidth(), getHeight()/2);
			lonField.setBounds(0, getHeight()/2, getWidth(), getHeight()/2);
		}
	}

	/**
	 * Saves the waypoint list to a file in JSON format
	 */
	private class WaypointSaver extends JButton {

		WaypointSaver(int height) {
			setSize(WAYPOINT_EDITOR_PANEL_WIDTH, height);
			setText("Save to file");
			addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					Gson translator = new GsonBuilder().excludeFieldsWithModifiers(Modifier.TRANSIENT).create();
					File outputFolder = new File(RobobuggyConfigFile.LOG_FILE_LOCATION + "/waypoints/");
					if (!outputFolder.exists() && !outputFolder.mkdirs()) {
						new RobobuggyLogicNotification("couldnt write waypoint list to file!", RobobuggyMessageLevel.WARNING);
						return;
					}
					PrintStream writer = null;
					try {
						writer = new PrintStream(new File(outputFolder.getPath() + "/waypoints.txt"), "UTF-8");
						for (MapMarker mapMarker : mapTree.getViewer().getMapMarkerList()) {
							//initialize a garbage holder for the lat and lon
							GpsMeasurement m = new GpsMeasurement(
											new Date(),
											new Date(),
											mapMarker.getLat(),
											true,
											mapMarker.getLon(),
											true, 0, 0, 0, 0);
							String serialize = translator.toJson(m);
							writer.println(serialize);
						}
						writer.close();

						new RobobuggyLogicNotification("Finished writing waypoint list to file", RobobuggyMessageLevel.NOTE);
					} catch (IOException e1) {
						e1.printStackTrace();
						new RobobuggyLogicNotification("couldnt write waypoint list to file!", RobobuggyMessageLevel.WARNING);
					}
				}
			});
		}

	}

	@Override
	public void paintComponent(Graphics g) {

		mapTree.setBounds(WAYPOINT_EDITOR_PANEL_WIDTH, 0, getWidth() - WAYPOINT_EDITOR_PANEL_WIDTH, getHeight());
		mapTree.getViewer().setSize(getWidth() - WAYPOINT_EDITOR_PANEL_WIDTH, getHeight());

		waypointListContainer.setBounds(
				0,
				0,
				WAYPOINT_EDITOR_PANEL_WIDTH,
				getHeight() - WAYPOINT_EDITOR_PANEL_HEIGHT - WAYPOINT_SAVER_PANEL_HEIGHT
		);
		waypointEditor.setBounds(
				0,
				getHeight() - WAYPOINT_EDITOR_PANEL_HEIGHT - WAYPOINT_SAVER_PANEL_HEIGHT,
				WAYPOINT_EDITOR_PANEL_WIDTH,
				WAYPOINT_EDITOR_PANEL_HEIGHT
		);
		waypointSaver.setBounds(
				0,
				getHeight() - WAYPOINT_SAVER_PANEL_HEIGHT,
				WAYPOINT_EDITOR_PANEL_WIDTH,
				WAYPOINT_SAVER_PANEL_HEIGHT
		);
	}
}
