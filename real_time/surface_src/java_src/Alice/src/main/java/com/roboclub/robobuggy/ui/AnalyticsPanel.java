package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.JPanel;

public class AnalyticsPanel extends RoboBuggyGUIContainer {
	private static final long serialVersionUID = 7017667286491619492L;

	private DataPanel dataPanel;
	private GraphPanel graphPanel;
	
	public AnalyticsPanel() {
		name = "analytics";
		dataPanel = new DataPanel();
		graphPanel = new GraphPanel();
		this.addComponet(dataPanel, 0, 0, 1, .6);
		this.addComponet(graphPanel, 0, .6, 1, .4);

	}
	
	public String valuesFromData()
	{
	  return dataPanel.getValues();	
	}
	
}
