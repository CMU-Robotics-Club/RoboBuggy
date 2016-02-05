package com.roboclub.robobuggy.ui;

/**
 * 
 * @author Trevor Decker
 *
 */
public class MainGuiWindow extends RobobuggyGUIContainer{
	
	/**
	 * constructor for the main gui window, sets up what is shown on the window
	 */
	public MainGuiWindow(){
		AnalyticsPanel analyPane = new AnalyticsPanel();
		ControlPanel ctrlPanel = new ControlPanel();
		//addComponent syntax is (newComponent,percentageLeft,percentageTop,percentageWidth,percentageHeight)
		addComponent(ctrlPanel, 0.0, 0.0, .3, 1.0);
		addComponent(analyPane, 0.3, 0.0, .7, 1.0);
	}

}
