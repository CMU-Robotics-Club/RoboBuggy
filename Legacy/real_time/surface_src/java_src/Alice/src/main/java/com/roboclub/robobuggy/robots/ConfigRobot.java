package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.ui.ConfigurationPanel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.PathPanel;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;

/**
 * Runs playback
 *
 * @author Trevor Decker
 */
public final class ConfigRobot extends AbstractRobot {

    private static ConfigRobot instance;

    /**
     * Returns a reference to the one instance of the {@link Robot} object.
     * If no instance exists, a new one is created.
     *
     * @return a reference to the one instance of the {@link Robot} object
     */
    public static AbstractRobot getInstance() {
        if (instance == null) {
            instance = new ConfigRobot();
        }
        return instance;
    }

    private ConfigRobot() {
        super();

        RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow", 1.0, 1.0);
        Gui.getInstance().addWindow(mainWindow);
        RobobuggyGUITabs tabs = new RobobuggyGUITabs();
        tabs.addTab(new ConfigurationPanel(), "Configuration");
        tabs.addTab(new PathPanel(), "Path Visualizer");
        mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
        Gui.getInstance().fixPaint();
    }
}	

