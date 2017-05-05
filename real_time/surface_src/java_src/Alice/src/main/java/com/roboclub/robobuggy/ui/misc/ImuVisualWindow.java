package com.roboclub.robobuggy.ui.misc;

import com.roboclub.robobuggy.ui.RobobuggyGUIContainer;

/**
 * Creates a window to display the IMU visualization
 *
 * @author davidneiman
 */
public class ImuVisualWindow extends RobobuggyGUIContainer {

    /**
     * Creates an ImuVisualWindow
     */
    public ImuVisualWindow() {
        ImuVisualPanel imuVP = new ImuVisualPanel();
        addComponent(imuVP, 0, 0, 1, 1);
    }

}
