package com.roboclub.robobuggy.ui;

import sun.security.jca.GetInstance;
import sun.security.jca.GetInstance.Instance;

/**
 * {@link RobobuggyGUIContainer} used to display a {@link DataPanel} and a
 *  {@link GraphPanel}
 */
public final class AnalyticsPanel extends RobobuggyGUIContainer {

	private static final long serialVersionUID = 7017667286491619492L;

	private DataPanel dataPanel;
	private GraphPanel graphPanel;
	private static AnalyticsPanel instance  = null;
	
	/**
	 * 
	 * @return a reference to the analytics panel 
	 */
	public static synchronized AnalyticsPanel getInstance(){
		if(instance == null){
			instance = new AnalyticsPanel();
		}
		return instance;
		
	}
	
	/**
	 * Construct a new {@link AnalyticsPanel}
	 */
	private AnalyticsPanel() {
		name = "analytics";
		dataPanel = new DataPanel();
		graphPanel = new GraphPanel();
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
