package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.nodes.localizers.RobobuggyKFLocalizer;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.simulation.LineByLineSensorPlayer;
import com.roboclub.robobuggy.ui.AutonomousPanel;
import com.roboclub.robobuggy.ui.ConfigurationPanel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.MainGuiWindow;
import com.roboclub.robobuggy.ui.PathPanel;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;

import java.io.FileNotFoundException;

/**
 * Runs playback
 *
 * @author Trevor Decker
 */
public final class PlayBackRobot extends AbstractRobot {

    private static PlayBackRobot instance;

    /**
     * Returns a reference to the one instance of the {@link Robot} object.
     * If no instance exists, a new one is created.
     *
     * @return a reference to the one instance of the {@link Robot} object
     */
    public static AbstractRobot getInstance() {
        if (instance == null) {
            instance = new PlayBackRobot();
        }
        return instance;
    }

    private PlayBackRobot() {
        super();
        new LineByLineSensorPlayer(RobobuggyConfigFile.getPlayBackSourceFile(), 1);
//        try {
//            nodeList.add(new RobobuggyKFLocalizer(10, "Robobuggy KF Localizer", new LocTuple(40.441670, -79.9416362)));
//            nodeList.add(new WayPointFollowerPlanner(WayPointUtil.createWayPointsFromWaypointList(RobobuggyConfigFile.getWaypointSourceLogFile())));
//        } catch (FileNotFoundException e) {
//            e.printStackTrace();
//        }
//		new SensorPlayer(RobobuggyConfigFile.getPlayBackSourceFile(), 1);
//		new HighTrustGPSLocalizer();
        RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow", 1.0, 1.0);
        Gui.getInstance().addWindow(mainWindow);
        RobobuggyGUITabs tabs = new RobobuggyGUITabs();
        mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
        tabs.addTab(new MainGuiWindow(), "Home");
        //	tabs.addTab(new PoseGraphsPanel(),"poses");
        tabs.addTab(new AutonomousPanel(), "Autonomous");
        tabs.add(new PathPanel(), "Path Visualizer");
        tabs.addTab(new ConfigurationPanel(), "Configuration");
    }
}	

