package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.BatteryLevelMessage;
import com.roboclub.robobuggy.messages.BrakeStateMessage;
import com.roboclub.robobuggy.messages.FingerPrintMessage;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;

/**
 * Created by vivaanbahl on 4/1/16.
 */
public class BuggyStatusPanel extends RobobuggyGUIContainer {

    private boolean brakesDown;
    private int batteryLevel;
    private String fphash = "(not loaded yet)";

    /**
     * initializes a new Buggy status panel
     */
    public BuggyStatusPanel() {

        setupDataLoaders();

    }

    private void setupDataLoaders() {
        new Subscriber(NodeChannel.BRAKE_STATE.getMsgPath(), (topicName, m) -> {
            brakesDown = ((BrakeStateMessage) m).isDown();
        });

        new Subscriber(NodeChannel.BATTERY.getMsgPath(), (topicName, m) -> {
            batteryLevel = ((BatteryLevelMessage) m).getBatteryLevel();
        });

        new Subscriber(NodeChannel.FP_HASH.getMsgPath(), ((topicName, m) -> {
            fphash = String.format("%x", ((FingerPrintMessage) m).getFpHash());
        }));
    }


    @Override
    protected void paintComponent(Graphics f) {
        super.paintComponent(f);

        int brakeX = 50;
        int brakeY = 0;
        String status = "up";

        int battLevelBoxLeft = brakeX + getHeight()/2;

        if (brakesDown) {
            brakeY = getHeight() - getHeight()/3;
            status = "down";
        }

        Graphics2D g = (Graphics2D)f;

        g.setColor(Color.RED);
        g.fillOval(brakeX, brakeY, getHeight()/3, getHeight()/3);


        g.setColor(Color.GREEN);
        g.fillRect(brakeX + getHeight()/3 + 10, 0, getHeight()/3, getHeight());
        g.setColor(Color.BLACK);
        g.setFont(new Font("Arial", Font.BOLD, 30));
        g.drawString("batt = " + batteryLevel, battLevelBoxLeft, getHeight()/2);
        g.drawString("status = " + status, 0, getHeight()/2);
        g.drawString("fphash = " + fphash, battLevelBoxLeft, 30);

    }
}
