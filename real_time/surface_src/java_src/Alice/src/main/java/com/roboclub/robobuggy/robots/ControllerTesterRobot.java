package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.nodes.planners.PolynomialPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.simulation.ControllerTester;
import com.roboclub.robobuggy.ui.ConfigurationPanel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.MainGuiWindow;
import com.roboclub.robobuggy.ui.PathPanel;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;

import java.io.IOException;
import java.util.ArrayList;

/**
 * Created by vivaanbahl on 1/26/17.
 */
public class ControllerTesterRobot extends AbstractRobot {
    private static ControllerTesterRobot instance;

    public static ControllerTesterRobot getInstance() {
        if (instance == null) {
            instance = new ControllerTesterRobot();
        }
        return instance;
    }

    private ControllerTesterRobot() {
        super();

        ArrayList<GpsMeasurement> wayPoints = null;
        try {
            wayPoints = WayPointUtil.createWayPointsFromWaypointList(RobobuggyConfigFile.getWaypointSourceLogFile());
            nodeList.add(new PolynomialPlanner(wayPoints));
//            nodeList.add(new WayPointFollowerPlanner(wayPoints));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        nodeList.add(new ControllerTester("Testing the controller", new LocTuple(wayPoints.get(0).getLatitude(), wayPoints.get(0).getLongitude())));

        //setup the gui
        RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow", 1.0, 1.0);
        Gui.getInstance().addWindow(mainWindow);
        RobobuggyGUITabs tabs = new RobobuggyGUITabs();
        mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
        tabs.addTab(new MainGuiWindow(), "Home");
        //	tabs.addTab(new PoseGraphsPanel(),"poses");
        //	tabs.addTab(new  AutonomousPanel(),"Autonomous");
        tabs.add(new PathPanel(), "Path Visualizer");
        tabs.addTab(new ConfigurationPanel(), "Configuration");
    }
}
