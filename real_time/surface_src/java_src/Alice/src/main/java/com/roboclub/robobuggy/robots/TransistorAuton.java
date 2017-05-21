package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.nodes.localizers.RobobuggyKFLocalizer;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ui.ConfigurationPanel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.MainGuiWindow;
import com.roboclub.robobuggy.ui.PathPanel;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;

import java.io.IOException;
import java.util.ArrayList;

import static com.roboclub.robobuggy.main.RobobuggyConfigFile.*;
/**
 * A robot class for having transistor drive itself
 *
 * @author Trevor Decker
 */
public final class TransistorAuton extends AbstractRobot {
    private static TransistorAuton instance;
    private static final int ARDUINO_BOOTLOADER_TIMEOUT = 2000;

    /**
     * Returns a reference to the one instance of the {@link} object.
     * If no instance exists, a new one is created.
     *
     * @return a reference to the one instance of the {@link } object
     */
    public static AbstractRobot getInstance() {
        if (instance == null) {
            instance = new TransistorAuton();
        }
        return instance;
    }

    /**
     * Constructor for TransistorAuton robot class
     */
    private TransistorAuton() {
        super();
        try {
            Thread.sleep(ARDUINO_BOOTLOADER_TIMEOUT);
        } catch (InterruptedException e) {
            new RobobuggyLogicNotification("Couldn't wait for bootloader, shutting down", RobobuggyMessageLevel.EXCEPTION);
            shutDown();
        }
        new RobobuggyLogicNotification("Logic Exception Setup properly", RobobuggyMessageLevel.NOTE);

        // Initialize Nodes
        nodeList.add(new RobobuggyKFLocalizer(10, "Robobuggy KF Localizer", new LocTuple(INITIAL_POS_LAT, INITIAL_POS_LON)));
        nodeList.add(new GpsNode(NodeChannel.GPS, RobobuggyConfigFile.getComPortGPS()));
        nodeList.add(new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, RobobuggyConfigFile.LOG_FILE_LOCATION,
                NodeChannel.getLoggingChannels()));
        nodeList.add(new RBSMNode(NodeChannel.ENCODER, NodeChannel.STEERING, RobobuggyConfigFile.getComPortRBSM(),
                RobobuggyConfigFile.RBSM_COMMAND_PERIOD));
        try {
            ArrayList<GpsMeasurement> wayPoints = WayPointUtil.createWayPointsFromWaypointList(RobobuggyConfigFile.getWaypointSourceLogFile());
            nodeList.add(new WayPointFollowerPlanner(wayPoints));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        //setup the gui
        RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow", 1.0, 1.0);
        Gui.getInstance().addWindow(mainWindow);
        RobobuggyGUITabs tabs = new RobobuggyGUITabs();
        mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
        tabs.addTab(new MainGuiWindow(), "Home");
        tabs.add(new PathPanel(), "Path Visualizer");
        tabs.addTab(new ConfigurationPanel(), "Configuration");


    }
}
