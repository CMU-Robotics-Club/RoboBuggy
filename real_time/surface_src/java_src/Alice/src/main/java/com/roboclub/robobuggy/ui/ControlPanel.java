package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JFormattedTextField;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.Timer;
import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.main.Config;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * {@link JPanel} used to represent a control pattern
 * @author Trevor Decker
 * @author Kevin Brennan
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class ControlPanel extends JPanel {
	private static final long serialVersionUID = -924045896215455343L;
	private static final int TIME_ZONE_OFFSET = 18000000;//5 hours

	private JButton playBtn;
	private JFormattedTextField timeLbl;
	private Date startTime;
	private Timer timer;

	private SensorSwitch gpsSwitch;
	private SensorSwitch visionSwitch;
	private SensorSwitch encodersSwitch;
	private SensorSwitch imuSwitch;
	private SensorSwitch controlsSwitch;
	private SensorSwitch autonomousSwitch;
	private JButton display;

	private Publisher loggingButtonPub;
	
	/**
	 * Construct a new {@link ControlPanel} object
	 */
	public ControlPanel() {
		loggingButtonPub = new Publisher(Gui.GuiPubSubTopics.GUI_LOG_BUTTON_UPDATED.toString());
		
		timer = new Timer(10, new TimerHandler());// updates every .01 seconds
		timer.setDelay(100);
		timer.setRepeats(true); // timer needs to be setup before startpause_btn

		//should be the time that we start the sytem at 
		startTime = new Date();
		
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridBagLayout());
		this.setBackground(Color.DARK_GRAY);

		GridBagConstraints gbc = new GridBagConstraints();
		gbc.weightx = 1;
		gbc.weighty = 0.5;
		gbc.gridx = 0;
		gbc.gridy = 0;
		gbc.fill = GridBagConstraints.BOTH;
		addLoggingPanel(gbc);
		
		gbc.gridy = 1;
		addSensorSwitchPanel(gbc);
	}

	/**
	 * Private class used to handle timer events
	 */
	private class TimerHandler implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent ae) {
			Date currentTime = new Date();
			timeLbl.setValue(currentTime.getTime() - startTime.getTime()+ TIME_ZONE_OFFSET);
		}
	}

	/**
	 * Private class used to handle play button actions
	 */
	private class PlayButtonHandler implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			// inverts the state of the system every time the button is pressed
			Config.active = !Config.active;
			
			if (Config.active) {
				System.out.println("System Started");
				playBtn.setBackground(Color.RED);
				playBtn.setText("STOP");
				timer.start();
				
				RobotLogger.createLog();
				loggingButtonPub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.START));
				startTime = new Date();
			} else {
				System.out.println("System Paused");
				playBtn.setBackground(Color.GREEN);
				playBtn.setText("START");
				
				RobotLogger.closeLog();
				loggingButtonPub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.STOP));
				timer.stop();
			}
		}
	}

	private void addLoggingPanel(GridBagConstraints gbc) {
		JPanel loggingPanel = new JPanel();
		loggingPanel.setBorder(BorderFactory.createLineBorder(Color.black));
		loggingPanel.setLayout(new GridBagLayout());
		
		GridBagConstraints gbcPanel = new GridBagConstraints();
		gbc.weightx = 1.0;
		
		playBtn = new JButton("START");
		playBtn.setFont(new Font("serif", Font.PLAIN, 70));
		playBtn.addActionListener(new PlayButtonHandler());
		playBtn.setEnabled(false);
		playBtn.setBackground(Color.BLUE);
		
		JLabel filenameLbl = new JLabel("File: ",
				SwingConstants.CENTER);
		filenameLbl.setFont(new Font("sanserif", Font.PLAIN, 30));

		timeLbl = new JFormattedTextField(new SimpleDateFormat("HH:mm:ss.S"));
		timeLbl.setHorizontalAlignment(SwingConstants.CENTER);
		timeLbl.setFont(new Font("sanserif", Font.PLAIN, 50));
		timeLbl.setEditable(false);
		timeLbl.setColumns(7);
		timeLbl.setValue(startTime);

		gbcPanel.weightx = 1;
		gbcPanel.fill = GridBagConstraints.BOTH;
		gbcPanel.gridx = 0;
		gbcPanel.gridy = 0;
		gbcPanel.weighty = 0.5;
		loggingPanel.add(playBtn, gbcPanel);
		
		gbcPanel.gridy = 1;
		gbcPanel.weighty = 0.25;
		loggingPanel.add(filenameLbl, gbcPanel);

		gbcPanel.gridy = 2;
		gbcPanel.weighty = 0.25;
		loggingPanel.add(timeLbl, gbcPanel);
		
		this.add(loggingPanel, gbc);
	}
	
	private void addSensorSwitchPanel(GridBagConstraints gbc) {
		JPanel switchPanel = new JPanel();
		switchPanel.setBorder(BorderFactory.createLineBorder(Color.black));
		switchPanel.setLayout(new GridLayout(7,1));
		
		gpsSwitch = new SensorSwitch("GPS", NodeChannel.GPS);
		visionSwitch = new SensorSwitch("VISION", NodeChannel.VISION);
		encodersSwitch = new SensorSwitch("ENCODERS", NodeChannel.ENCODER);
		imuSwitch = new SensorSwitch("IMU", NodeChannel.IMU);
		controlsSwitch = new SensorSwitch("CONTROLS", NodeChannel.DRIVE_CTRL);
		autonomousSwitch = new SensorSwitch("AUTO", NodeChannel.AUTO);
		
		display = new JButton("DISPLAY");
		display.setBackground(Color.BLUE);
		display.setForeground(Color.WHITE);
		display.setEnabled(false);
		display.setFont(new Font("serif", Font.BOLD, 20));
		display.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				// TODO Auto-generated method stub
			}
		});
		
		switchPanel.add(autonomousSwitch);
		switchPanel.add(gpsSwitch);
		switchPanel.add(imuSwitch);
		switchPanel.add(encodersSwitch);
		switchPanel.add(controlsSwitch);
		switchPanel.add(visionSwitch);
		switchPanel.add(display);
		
		this.add(switchPanel, gbc);
	}
	
	/**
	 * Enables logging
	 */
	public void enableLogging() {
		playBtn.setEnabled(true);
		playBtn.setBackground(Color.GREEN);
	}
}
