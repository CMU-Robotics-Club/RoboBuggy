package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Container;
import java.awt.Frame;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.WindowEvent;
import java.io.File;

import javax.imageio.ImageIO;
import javax.swing.JFrame;

import com.roboclub.robobuggy.main.Robot;

/**
 * {@link JFrame} used to represent the robobuggy gui
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
	
	/**
	 * Enum of {@link Gui} publisher and subscriber topics
	 */
	public enum GuiPubSubTopics {
		GUI_LOG_BUTTON_UPDATED,
	}

	/**
	 * Returns a reference to the one instance of the {@link Gui}. 
	 * If none exists, one will be constructed.
	 * @return a reference to the one instance of the {@link Gui}
	 */
	public static Gui getInstance() {
		if (instance == null) {
			instance = new Gui();
		}
		return instance;
	}

	/**
	 * Construct a new {@link Gui} object
	 */
	private Gui() {
		System.out.println("Starting GUI");
		populate();
		System.out.println("Populated!");
	}

	private void populate() {
		this.setTitle("RoboBuggy Interface");
		this.setExtendedState(Frame.MAXIMIZED_BOTH);
		Container pane = this.getContentPane();
		pane.setLayout(new GridBagLayout());
		pane.setBackground(Color.DARK_GRAY);

		try {
			this.setIconImage(ImageIO.read(new File("images/rc_logo.png")));
		} catch (Exception e) {
			System.out.println("Unable to read icon image!");
		}
		
		// Close ports and close window upon exit
		this.addWindowListener(new java.awt.event.WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent windowEvent) {
				try {
					Robot.getInstance().shutDown();
				}
				catch(NullPointerException e) {
					e.printStackTrace();
				}
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
		gbc.gridy = 0;
		anlyPanel = new AnalyticsPanel();
		pane.add(anlyPanel, gbc);
		
		
		this.pack();
		this.setVisible(true);
		
	}
	
	/**
	 * Closes the {@link Gui}
	 */
	public static void close() {
		System.out.println("Trying to close the window!");
		instance.dispatchEvent(new WindowEvent(instance, WindowEvent.WINDOW_CLOSING));
		System.out.println("Apparently we've closed the window!");
	}

	/**
	 * Enables logging
	 */
	public static void enableLogging() {
		instance.ctrlPanel.enableLogging();
	}
}