package com.roboclub.robobuggy.ui;

import java.awt.Container;
import java.awt.Frame;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.io.File;
import javax.imageio.ImageIO;
import javax.swing.JFrame;
import com.roboclub.robobuggy.main.Robot;

/**
 * 
 * @author Trevor Decker
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public final class Gui extends JFrame {
	private static final long serialVersionUID = 670947948979376738L;

	private ControlPanel ctrlPanel;
	private AnalyticsPanel anlyPanel;
	
	private static Gui instance;

	public static Gui getInstance() {
		if (instance == null) {
			instance = new Gui();
		}
		return instance;
	}

	public Gui() {
		System.out.println("Starting GUI");
		populate();
	}

	public void populate() {
		this.setTitle("RoboBuggy Interface");
		this.setExtendedState(Frame.MAXIMIZED_BOTH);
		Container pane = this.getContentPane();
		pane.setLayout(new GridBagLayout());

		try {
			this.setIconImage(ImageIO.read(new File("images/rc_logo.png")));
		} catch (Exception e) {
			System.out.println("Unable to read icon image!");
		}
		
		// Close ports and close window upon exit
		this.addWindowListener(new java.awt.event.WindowAdapter() {
			@Override
			public void windowClosing(java.awt.event.WindowEvent windowEvent) {
				Robot.ShutDown();
			}
		});
		
		GridBagConstraints gbc = new GridBagConstraints();
		gbc.weightx = 0.25;
		gbc.weighty = 1.0;
		gbc.gridx = 0;
		gbc.gridy = 0;
		gbc.fill = GridBagConstraints.BOTH;
		ctrlPanel = new ControlPanel();
		pane.add(ctrlPanel, gbc);
		
		gbc.weightx = 0.75;
		gbc.gridx = 1;
		anlyPanel = new AnalyticsPanel();
		pane.add(anlyPanel, gbc);
		
		this.pack();
		this.setVisible(true);
	}

	public static void EnableLogging() {
		instance.ctrlPanel.enableLogging();
	}
}