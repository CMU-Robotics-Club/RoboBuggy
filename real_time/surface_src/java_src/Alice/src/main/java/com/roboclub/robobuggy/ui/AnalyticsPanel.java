package com.roboclub.robobuggy.ui;

/**
 * {@link RobobuggyGUIContainer} used to display a {@link DataPanel} and a
 *  {@link GraphPanel}
 */
public class AnalyticsPanel extends RobobuggyGUIContainer {

	private static final long serialVersionUID = 7017667286491619492L;

	private DataPanel dataPanel;
//	private GraphPanel graphPanel;
	BuggyStatusPanel graphPanel;
	
	/**
	 * Construct a new {@link AnalyticsPanel}
	 */
	public AnalyticsPanel() {
		name = "analytics";
		dataPanel = new DataPanel();
		graphPanel = new BuggyStatusPanel();
		this.addComponent(dataPanel, 0, 0, 1, .6);
		this.addComponent(graphPanel, 0, .6, 1, .4);

	}
	
	/**
	 * Returns data values from the {@link DataPanel}
	 * @return data values from the {@link DataPanel}
	 */
	public String valuesFromData()
	{
	  return dataPanel.getValues();	
	}


	/**
	 * @return the data panel
	 */
	public DataPanel getDataPanel() {
		return dataPanel;
	}
}
