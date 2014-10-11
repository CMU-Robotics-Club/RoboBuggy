package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.imageio.ImageIO;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.Timer;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.main.config;

public final class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;

	private static JCheckBox toggleGraph;
	private static JTextArea filename;
	private static AnalyticsWindow graphs;
	private static Timer timer;
	private static long startTime;
	private static Gui instance;
	private static boolean graphDisplay;
	public static Container mainGuiPane;

	public static Gui getInstance() {
		if (instance == null) {
			instance = new Gui();
		}
		return instance;
	}

	public Gui() {
		System.out.println("starting the GUI \n");
		populate();
	}

	public void populate() {
		this.setTitle("RoboBuggy Data Gathering");
		this.setResizable(false);
		mainGuiPane = this;
		Container pane = this.getContentPane();
		pane.setLayout(new BoxLayout(pane, BoxLayout.Y_AXIS));
		

		try {
			this.setIconImage(ImageIO.read(new File("images/rc_logo.png")));
		} catch (Exception e) {
			System.out.println("Unable to read icon image!");
		}
		
		

		// Initialize Analytics Window
		graphs = new AnalyticsWindow();

		// Add items to window
		pane.add(addToggleBtn());
		pane.add(addTimeShow());
		pane.add(addFileShow());
		pane.add(addGraphShow());

		// Close ports and close window upon exit
		this.addWindowListener(new java.awt.event.WindowAdapter() {
			@Override
			public void windowClosing(java.awt.event.WindowEvent windowEvent) {
				Robot.ShutDown();
			}
		});

		JButton resetEncoders_btn = new JButton("Reset Encoders");
		resetEncoders_btn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO
			}});
		pane.add(resetEncoders_btn);
		
		JButton resetGPS_btn = new JButton("Reset GPS");
		resetEncoders_btn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO
			}});
		pane.add(resetGPS_btn);
		
		JButton recenterServo_btn = new JButton("Recenter Servo");
		recenterServo_btn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO
			}});
		pane.add(recenterServo_btn);
		
		JButton resetIMU_btn = new JButton("Reset IMU");
		resetIMU_btn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO
			}});
		pane.add(resetIMU_btn);
		
		JButton resetVision_btn = new JButton("Reset Vision");
		resetVision_btn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO
			}});
		pane.add(resetVision_btn);
		
		JButton resetLocalization_btn = new JButton("Reset localization");
		resetLocalization_btn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO
			}});
		pane.add(resetLocalization_btn);
		
		JButton NewLog_btn = new JButton("New Log");
		resetEncoders_btn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO
			}});
		pane.add(NewLog_btn);		
		
		this.pack();
		this.setAlwaysOnTop(true);
		this.setVisible(true);
	}

	static JButton playPauseButton;

	private JButton addToggleBtn() {
		playPauseButton = new JButton();
		playPauseButton.setPreferredSize(new Dimension(WIDTH, 100));
		playPauseButton.setFont(new Font("sanserif", Font.PLAIN, 50));
		playPauseButton.setText("Start");
		playPauseButton.setSelected(config.active);
		if (config.active) {
			playPauseButton.setBackground(Color.RED);
		} else {
			playPauseButton.setBackground(Color.GREEN);
		}

		playPauseButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				config.active = !config.active;

				if (config.active) {
					playPauseButton.setText("Pause");
					playPauseButton.setBackground(Color.RED);

					RobotLogger.CreateLog();

					startTime = new Date().getTime();
					timer.start();
				} else {
					playPauseButton.setText("Start");
					playPauseButton.setBackground(Color.GREEN);
					timer.stop();

					RobotLogger.CloseLog();
				}
				ControlPanel.updateStartPause_btn();
				// AnalyticsWindow
			}
		});

		playPauseButton.setAlignmentX(Component.CENTER_ALIGNMENT);

		return playPauseButton;
	}

	public static void updatedPalyPause() {
		if (config.active) {
			playPauseButton.setText("Pause");
			playPauseButton.setBackground(Color.RED);
			startTime = new Date().getTime();
			RobotLogger.CreateLog();
			timer.start();
		} else {
			playPauseButton.setText("Start");
			playPauseButton.setBackground(Color.GREEN);
			timer.stop();

			RobotLogger.CloseLog();
		}

	}

	private JPanel addFileShow() {
		JPanel fileShow = new JPanel();

		JLabel file_lbl = new JLabel("Log:");
		file_lbl.setFont(new Font("sanserif", Font.PLAIN, 20));
		file_lbl.setAlignmentX(Component.LEFT_ALIGNMENT);
		fileShow.add(file_lbl);

		filename = new JTextArea();
		filename.setEditable(false);
		filename.setFont(new Font("sanserif", Font.PLAIN, 20));
		filename.setAlignmentX(Component.RIGHT_ALIGNMENT);
		fileShow.add(filename);

		return fileShow;
	}

	private JPanel addGraphShow() {
		JPanel graphShow = new JPanel();

		JLabel graph_lbl = new JLabel("Display Graphs:");
		graph_lbl.setFont(new Font("sanserif", Font.PLAIN, 20));
		graph_lbl.setAlignmentX(Component.LEFT_ALIGNMENT);
		graphShow.add(graph_lbl);

		toggleGraph = new JCheckBox();
		toggleGraph.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				graphDisplay = !graphDisplay;
				graphs.setVisible(graphDisplay);
				// turns visablity of main gui off since graphs is up
				mainGuiPane.setVisible(false);
			}
		});
		graphShow.add(toggleGraph);

		return graphShow;
	}

	private JLabel addTimeShow() {
		JLabel elapsedTime = new JLabel();
		elapsedTime.setFont(new Font("sanserif", Font.PLAIN, 30));
		DateFormat df = new SimpleDateFormat("hh:mm:ss.S");
		elapsedTime.setText(df.format(new Date(0)));

		timer = new Timer(100, new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				elapsedTime.setText(df.format(new Date(new Date().getTime()
						- startTime)));
			}
		});
		timer.setRepeats(true);

		elapsedTime.setAlignmentX(Component.CENTER_ALIGNMENT);

		return elapsedTime;
	}

	public static void updateGraphToggle(boolean value) {
		graphDisplay = value;

		toggleGraph.setSelected(graphDisplay);
		updatedPalyPause();

	}

	/* Methods for Statically Accessing Gui Instance Variables */
	public boolean GetGraphState() {
		return graphDisplay;
	}

	// Update Sensor Data in Graph
	public static void UpdateRobotPos(Float lat, Float lon) {
		/*
		 * if (lat != null) robot.latitude = lat; if (lon != null)
		 * robot.longitude = lon;
		 */

		// if (graphDisplay && graphs != null && graphs.data != null)
		// graphs.data.UpdatePos(lat, lon);
	}

	public void UpdateRobotAccel(Float aX, Float aY, Float aZ) {
		/*
		 * if (aX != null) robot.aX = aX; if (aY != null) robot.aY = aY; if (aZ
		 * != null) robot.aZ = aZ;
		 */

		// if (graphDisplay && graphs != null && graphs.data != null)
		// graphs.data.UpdateAccel(aX, aY, aZ);
	}

	public void UpdateRobotGyro(Float rX, Float rY, Float rZ) {
		/*
		 * if (rX != null) robot.rX = rX; if (rY != null) robot.rY = rY; if (rZ
		 * != null) robot.rZ = rZ;
		 */

		// if (graphDisplay && graphs != null && graphs.data != null)
		// graphs.data.UpdateGyro(rX, rY, rZ);
	}

	public void UpdateRobotMagnet(Float mX, Float mY, Float mZ) {
		/*
		 * if (mX != null) robot.mX = mX; if (mY != null) robot.mY = mY; if (mZ
		 * != null) robot.mZ = mZ;
		 */

		// if (graphDisplay && graphs != null && graphs.data != null)
		// graphs.data.UpdateMagnet(mX, mY, mZ);
	}

	public void UpdateRobotAngle(Integer ang) {
		if (graphDisplay && ang != null) {
			// robot.encAng = ang;

			// if (graphs != null && graphs.data != null)
			// graphs.data.UpdateAngle(ang);
		}
	}

	public void UpdateRobotDst(Long dst) {
		if (graphDisplay && dst != null) {
			// robot.encDst = dst;

			// if (graphs != null && graphs.data != null)
			// graphs.data.UpdateDst(dst);
		}
	}

	public void UpdateLogName(String filename_) {
		if (filename != null) {
			filename.setText(filename_);
		}
	}
}