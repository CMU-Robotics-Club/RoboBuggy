package com.roboclub.robobuggy.ui;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import org.openstreetmap.gui.jmapviewer.*;
import org.openstreetmap.gui.jmapviewer.interfaces.ICoordinate;
import org.openstreetmap.gui.jmapviewer.interfaces.MapMarker;
import org.openstreetmap.gui.jmapviewer.interfaces.TileCache;
import org.openstreetmap.gui.jmapviewer.tilesources.BingAerialTileSource;

import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.PrintStream;
import java.lang.reflect.Modifier;
import java.util.Date;
import java.util.List;

/**
 * Created by vivaanbahl on 3/17/16.
 */
public class PathEditorTab extends RobobuggyGUIContainer {

	private static final int WAYPOINT_EDITOR_PANEL_WIDTH = 200;
	private static final int WAYPOINT_EDITOR_PANEL_HEIGHT = 100;
	private static final int WAYPOINT_SAVER_PANEL_HEIGHT = 50;

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


	public PathEditorTab() {

		initMapTree();
		addCacheToTree();

		this.add(mapTree);

		
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


	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);

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

					mapTree.getViewer().setDisplayPosition(new Coordinate(dot.getLat(), dot.getLon()), mapTree.getViewer().getZoom());
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

}
