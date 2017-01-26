package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.simulation.ControllerTester;

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

        nodeList.add(new ControllerTester("Testing the controller"));
    }
}
