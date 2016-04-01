package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.BrakeStateMessage;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import org.junit.Before;
import org.junit.Test;

/**
 * commenting this out until gui tests work on travis
 */
public class BuggyStatusPanelTest {
//
    @Before
    public void setup() {

        Gui.getInstance();

    }
//
//    @After
//    public void tearDown() {
//        Gui.close();
//    }
//
    @Test
    public void test_GuiBrakesToggle() throws InterruptedException {
        Publisher p = new Publisher(NodeChannel.BRAKE_STATE.getMsgPath());
        for (int i = 0; i < 10; i++) {
            Thread.sleep(1000);
            p.publish(new BrakeStateMessage(false));
            Thread.sleep(1000);
            p.publish(new BrakeStateMessage(true));
        }
    }
//
//    @Test
//    public void test_GuiBatteryLevelIncrease() throws InterruptedException {
//        Publisher p2 = new Publisher(NodeChannel.BATTERY.getMsgPath());
//        Thread.sleep(1000);
//        for (int i = 0; i < 10; i++) {
//            p2.publish(new BatteryLevelMessage(i * 1000));
//        }
//    }

}