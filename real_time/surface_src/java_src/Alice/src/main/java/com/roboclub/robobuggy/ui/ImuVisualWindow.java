package com.roboclub.robobuggy.ui;

public class ImuVisualWindow extends RobobuggyGUIContainer{

	public ImuVisualWindow(){
		ImuVisualPanel ImuVP = new ImuVisualPanel();
		addComponent(ImuVP, 0, 0, 1, 1);
	}

}
