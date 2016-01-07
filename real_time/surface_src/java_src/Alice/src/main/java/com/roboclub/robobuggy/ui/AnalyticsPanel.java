package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.JPanel;

/**
 * {@link JPanel} used to display a {@link DataPanel} and a {@link GraphPanel}
 */
public class AnalyticsPanel extends JPanel {
	private static final long serialVersionUID = 7017667286491619492L;

	private DataPanel dataPanel;
	private GraphPanel graphPanel;
	
	/**
	 * Construct a new {@link AnalyticsPanel}
	 */
	public AnalyticsPanel() {
		this.setLayout(new GridBagLayout());
		this.setBackground(Color.DARK_GRAY);
		
		GridBagConstraints gbc = new GridBagConstraints();
		
		dataPanel = new DataPanel();
		graphPanel = new GraphPanel();
		
		gbc.fill = GridBagConstraints.BOTH;
		gbc.weighty = 1;
		gbc.weightx = 1;
		gbc.gridx = 0;
		gbc.gridy = 0;
		gbc.anchor = GridBagConstraints.LINE_START;
		this.add(dataPanel, gbc);
		
		gbc.gridy = 1;
		gbc.weighty = 0;
		this.add(graphPanel, gbc);
	    System.out.println("H");
	}
	
	/**
	 * Returns data values from the {@link DataPanel}
	 * @return data values from the {@link DataPanel}
	 */
	public String valuesFromData()
	{
	  return dataPanel.getValues();	
	}
	
}
