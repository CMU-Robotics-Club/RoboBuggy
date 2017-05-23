package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.nodes.planners.SweepNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.MainGuiWindow;


/**
 * Created by Robot on 1/24/2017.
 */
public final class CommTestRobot extends AbstractRobot {
    private static CommTestRobot instance;
    private static final int ARDUINO_BOOTLOADER_TIMEOUT_MS = 2000;

    /**
     * Returns a reference to the one instance of the {@link Robot} object.
     * If no instance exists, a new one is created.
     *
     * @return a reference to the one instance of the {@link Robot} object
     */
    public static CommTestRobot getInstance() {
        if (instance == null) {
            instance = new CommTestRobot();
        }
        return instance;
    }

    private CommTestRobot() {
        super();
        try {
            Thread.sleep(ARDUINO_BOOTLOADER_TIMEOUT_MS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        nodeList.add(new RBSMNode(NodeChannel.ENCODER, NodeChannel.STEERING,
                RobobuggyConfigFile.getComPortRBSM(), RobobuggyConfigFile.RBSM_COMMAND_PERIOD));
        nodeList.add(new SweepNode(NodeChannel.DRIVE_CTRL));

        //setup the gui
        RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow", 1.0, 1.0);
        Gui.getInstance().addWindow(mainWindow);
        RobobuggyGUITabs tabs = new RobobuggyGUITabs();
        mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
        tabs.addTab(new MainGuiWindow(), "Home");

    }

}
