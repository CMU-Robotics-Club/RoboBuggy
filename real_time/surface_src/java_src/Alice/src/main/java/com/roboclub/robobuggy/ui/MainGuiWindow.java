package com.roboclub.robobuggy.ui;

public class MainGuiWindow extends RobobuggyGUIContainer{
	
	public MainGuiWindow(){
		AnalyticsPanel analyPane = new AnalyticsPanel();
		ControlPanel ctrlPanel = new ControlPanel();
		//addComponent syntax is (newComponent,percentageLeft,percentageTop,percentageWidth,percentageHeight)
		addComponent(ctrlPanel, 0.0, 0.0, .3, 1.0);
		addComponent(analyPane, 0.3, 0.0, .7, 1.0);
	}

}
