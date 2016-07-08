package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.BatteryLevelMessage;
import com.roboclub.robobuggy.messages.BrakeStateMessage;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import org.junit.Test;

/**
 * commenting this out until gui tests work on travis
 */
public class BuggyStatusPanelTest {

    /**
     * Toggles the brakes at 1 hz
     *
     * @throws InterruptedException if ur stupid (lol jk) its just there for teh lolz
     *                              (and the fact that we put sleeps in)
     */
    @Test
    public void testGuiBrakesToggle() throws InterruptedException {
        if (!System.getProperty("os.name").equals("Mac OS X")) {
            System.out.println("skipping this test because you are not running mac OS X ");
            //this test fails on travis so we only want to run it locally
            return;
        }
        Gui.getInstance();
        Publisher p = new Publisher(NodeChannel.BRAKE_STATE.getMsgPath());
        for (int i = 0; i < 10; i++) {
            Thread.sleep(1000);
            p.publish(new BrakeStateMessage(false));
            Thread.sleep(1000);
            p.publish(new BrakeStateMessage(true));
        }
    }

    /**
     * Increases the battery at 1 hz by 1000
     *
     * @throws InterruptedException if ur stupid (lol jk) its just there for teh lolz
     *                              (and the fact that we put sleeps in)
     */
    @Test
    public void testGuiBatteryLevelIncrease() throws InterruptedException {
        if (!System.getProperty("os.name").equals("Mac OS X")) {
            System.out.println("skipping this test because you are not running mac OS X ");
            //this test fails on travis so we only want to run it locally
            return;
        }
        Gui.getInstance();
        Publisher p2 = new Publisher(NodeChannel.BATTERY.getMsgPath());
        Thread.sleep(1000);
        for (int i = 0; i < 10; i++) {
            p2.publish(new BatteryLevelMessage(i * 1000));
        }
    }

}