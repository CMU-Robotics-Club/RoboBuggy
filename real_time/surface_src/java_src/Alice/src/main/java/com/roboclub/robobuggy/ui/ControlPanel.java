package com.roboclub.robobuggy.ui;

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
		this.addComponent(loggingPane, 0, 0, 1, .3);
		this.addComponent(sensorsPane, 0, .3, 1, .3);
		this.addComponent(messageConsole,0,.6,1,.4);
		
	}

}
