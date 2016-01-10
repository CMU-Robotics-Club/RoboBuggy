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
import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;

/**
 * 
 * @author Trevor Decker
 * @author Kevin Brennan
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

public class ControlPanel extends RoboBuggyGUIContainer {
	private static final long serialVersionUID = -924045896215455343L;

	
	public ControlPanel() {

		LoggingPanel loggingPane = new LoggingPanel();
		SensorSwitchPanel sensorsPane = new SensorSwitchPanel();
		LogicErrorMessageConsole messageConsole = new LogicErrorMessageConsole();
		this.addComponet(loggingPane, 0, 0, 1, .3);
		this.addComponet(sensorsPane, 0, .3, 1, .3);
		this.addComponet(messageConsole,0,.6,1,.4);
		
	}


	

}
