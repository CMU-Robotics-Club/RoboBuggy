package com.roboclub.robobuggy.main;

import java.awt.Color;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.Timer;

public final class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;

	private static Gui instance;
	private static JCheckBox toggleGraph;
	private static ArrayList<CameraPanel> cameraPanels;
	private static AnalyticsWindow graphs;
	private Timer timer;
	private static long startTime;
	private static boolean playState;
	private static boolean graphDisplay;
	private static boolean cameraDisplay;
	
	//private static Robot robot;

	public static void main(String args[]) {
		// Check for commandline arguments
		ArrayList<Integer> cameras = new ArrayList<Integer>();
		for (int i = 0; i < args.length; i++) {
			if (i+1 < args.length) {
				if (args[i].equalsIgnoreCase("-c")) {
					cameras.add( Integer.valueOf(args[i+1]) );
				}
			}
		}
		
		Gui.getInstance().populate( cameras );
	}
	
	public static Gui getInstance(){
		if(instance == null){
			System.out.println("Created Gui instance");
			instance = new Gui();
		}
		return instance;
	}
	
	public Gui() {
		playState = false;
		graphDisplay = false;
		cameraDisplay = false;
	}
	
	public void populate(ArrayList<Integer> cameras) {
		this.setTitle("RoboBuggy Data Gathering");
		this.setResizable(false);
		Container pane = this.getContentPane();
		pane.setLayout(new BoxLayout(pane, BoxLayout.Y_AXIS));
		
		// Initialize Camera Feeds
		cameraPanels = new ArrayList<CameraPanel>();
		if (cameras != null && !cameras.isEmpty()) {
			for (Integer id : cameras) {
				try {
					cameraPanels.add(new CameraPanel( id ));
				} catch (Exception e) {
					System.out.println("Failed To Initialize Camera: " + id);
				}
			}
		}
		
		// Initialize Analytics Window
		graphs = new AnalyticsWindow();
		
		// Add items to window
		pane.add(addToggleBtn());
		pane.add(addTimeShow());
		pane.add(addFileShow());
		pane.add(addGraphShow());
		pane.add(addCameraShow());
		
		// Close ports and close window upon exit
		this.addWindowListener(new java.awt.event.WindowAdapter() {
		    @Override
		    public void windowClosing(java.awt.event.WindowEvent windowEvent) {
		    	closeWindow(0);
		    }
		});
		
		this.pack();
		this.setAlwaysOnTop(true);
		this.setVisible(true);
	}
	
	private JButton addToggleBtn() {
		JButton toggleButton = new JButton();
		toggleButton.setPreferredSize(new Dimension(WIDTH, 100));
		toggleButton.setFont(new Font("sanserif",Font.PLAIN,50));
		toggleButton.setText("Start");
		toggleButton.setSelected(playState);
		if (playState) {
			toggleButton.setBackground(Color.RED);
		} else {
			toggleButton.setBackground(Color.GREEN);
		}
		
		toggleButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				playState = !playState;
				
				if (playState) {
					toggleButton.setText("Pause");
					toggleButton.setBackground(Color.RED);
					startTime = new Date().getTime();
					timer.start();
				} else {
					toggleButton.setText("Start");
					toggleButton.setBackground(Color.GREEN);
					timer.stop();
				}
			}
		});
		
		toggleButton.setAlignmentX(Component.CENTER_ALIGNMENT);
		
		return toggleButton;
	}
	
	private JPanel addFileShow() {
		JPanel fileShow = new JPanel();
		
		JLabel file_lbl = new JLabel("Log:");
		file_lbl.setFont(new Font("sanserif",Font.PLAIN,20));
		file_lbl.setAlignmentX(Component.LEFT_ALIGNMENT);
		fileShow.add(file_lbl);
		
		JTextArea filename = new JTextArea();
		filename.setText("Filename here!!!");
		filename.setEditable(false);
		filename.setFont(new Font("sanserif",Font.PLAIN,20));
		filename.setAlignmentX(Component.RIGHT_ALIGNMENT);
		fileShow.add(filename);

		return fileShow;
	}
	
	private JPanel addGraphShow() {
		JPanel graphShow = new JPanel();
		
		JLabel graph_lbl = new JLabel("Display Graphs:");
		graph_lbl.setFont(new Font("sanserif",Font.PLAIN,20));
		graph_lbl.setAlignmentX(Component.LEFT_ALIGNMENT);
		graphShow.add(graph_lbl);
		
		toggleGraph = new JCheckBox();
		toggleGraph.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				graphDisplay = !graphDisplay;
				
				graphs.setVisible(graphDisplay);
			}
		});
		graphShow.add(toggleGraph);
		
		return graphShow;
	}
	
	private JLabel addTimeShow() {
		JLabel elapsedTime = new JLabel();
		elapsedTime.setFont(new Font("sanserif",Font.PLAIN,30));
		DateFormat df = new SimpleDateFormat("hh:mm:ss.S");
		elapsedTime.setText(df.format(new Date(0)));
		
		timer = new Timer(100, new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				elapsedTime.setText(
						df.format(new Date(new Date().getTime() - startTime)));
			}
		});
	    timer.setRepeats(true);
	    
	    elapsedTime.setAlignmentX(Component.CENTER_ALIGNMENT);
		
		return elapsedTime;
	}
	
	private JPanel addCameraShow() {
		JPanel cameraShow = new JPanel();
		
		JLabel displayCameras = new JLabel("Display Cameras");
		displayCameras.setFont(new Font("sanserif",Font.PLAIN,20));
		displayCameras.setAlignmentX(Component.LEFT_ALIGNMENT);
		cameraShow.add(displayCameras);
		
		JCheckBox toggleCameras = new JCheckBox();
		if (cameraPanels != null && !cameraPanels.isEmpty()) {
			toggleCameras.setSelected(cameraDisplay);
			toggleCameras.addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					cameraDisplay = !cameraDisplay;
					
					if (cameraPanels != null && !cameraPanels.isEmpty()) {
						for (CameraPanel panel : cameraPanels) {
							panel.setDisplaying(cameraDisplay);
						}
					}
				}
			});
		}
		else { 
			toggleCameras.setEnabled(false);
		}
		cameraShow.add(toggleCameras);
		
		return cameraShow;
	}
	
	private void closeWindow(int exitCode) {
		if (graphs != null) {
			graphs.close();
		}
		
		if (cameraPanels != null && !cameraPanels.isEmpty()) {
			for (CameraPanel panel : cameraPanels) {
				panel.close();
			}
		}
        System.exit(exitCode);
	}

	public static void updateGraphToggle(boolean value) {
		graphDisplay = value;
		
		toggleGraph.setSelected(graphDisplay);
	}
	
	/* Methods for Statically Accessing Gui Instance Variables */
	public static boolean GetPlayPauseState() {
		return playState;
	}

	public static void setPlayPauseState(boolean playPauseState) {
		Gui.playState = playPauseState;
		
		if (cameraPanels != null && !cameraPanels.isEmpty()) {
			for (CameraPanel panel : cameraPanels) {
				panel.setLogging(playPauseState);
			}
		}
	}
	
	public static boolean HasCameras() {
		return (cameraPanels != null && !cameraPanels.isEmpty());
	}
	
	public static boolean GetDisplayState() {
		return cameraDisplay;
	}
	
	public static boolean GetGraphState() {
		return graphDisplay;
	}

	// Update Sensor Data in Graph
	public static void UpdateRobotPos(Float lat, Float lon) {
		/*if (lat != null) robot.latitude = lat;
		if (lon != null) robot.longitude = lon; */
		
		if (graphDisplay && graphs != null && graphs.data != null) graphs.data.UpdatePos(lat, lon);
	}
	
	public static void UpdateRobotAccel(Float aX, Float aY, Float aZ) {
		/*if (aX != null) robot.aX = aX;
		if (aY != null) robot.aY = aY;
		if (aZ != null) robot.aZ = aZ;*/
		
		if (graphDisplay && graphs != null && graphs.data != null) graphs.data.UpdateAccel(aX, aY, aZ);
	}
	
	public static void UpdateRobotGyro(Float rX, Float rY, Float rZ) {
		/*if (rX != null) robot.rX = rX;
		if (rY != null) robot.rY = rY;
		if (rZ != null) robot.rZ = rZ;*/
		
		if (graphDisplay && graphs != null && graphs.data != null) graphs.data.UpdateGyro(rX, rY, rZ);
	}
	
	public static void UpdateRobotMagnet(Float mX, Float mY, Float mZ) {
		/*if (mX != null) robot.mX = mX;
		if (mY != null) robot.mY = mY;
		if (mZ != null) robot.mZ = mZ;*/
		
		if (graphDisplay && graphs != null && graphs.data != null) graphs.data.UpdateMagnet(mX, mY, mZ);
	}
	
	public static void UpdateRobotAngle(Integer ang) {
		if (graphDisplay && ang != null) {
			//robot.encAng = ang;
			
			if (graphs != null && graphs.data != null) graphs.data.UpdateAngle(ang);
		}
	}
	
	public static void UpdateRobotDst(Long dst) {
		if (graphDisplay && dst != null) {
			//robot.encDst = dst;
			
			if (graphs != null && graphs.data != null) graphs.data.UpdateDst(dst);
		}
	}
}