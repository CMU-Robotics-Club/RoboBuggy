package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.BatteryLevelMessage;
import com.roboclub.robobuggy.messages.BrakeStateMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.IMUAngularPositionMessage;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

import javax.imageio.ImageIO;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

/**
 * Created by vivaanbahl on 4/1/16.
 */
public class BuggyStatusPanel extends RobobuggyGUIContainer {

    private boolean brakesDown;
    private int batteryLevel;

    private BufferedImage imuArrow;
    private BufferedImage gpsArrow;

    private double imuAngle;
    private double gpsAngle;
    private GpsMeasurement prevGPS;

    /**
     * initializes a new Buggy status panel
     */
    public BuggyStatusPanel() {

        setupDataLoaders();

        try {
            imuArrow = ImageIO.read(new File("images/imuarrow.jpg"));
            imuArrow = toBufferedImage(imuArrow.getScaledInstance(150, 50, 0));

            gpsArrow = ImageIO.read(new File("images/gpsarrow.jpg"));
            gpsArrow = toBufferedImage(gpsArrow.getScaledInstance(150, 50, 0));
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    private void setupDataLoaders() {
        new Subscriber(NodeChannel.BRAKE_STATE.getMsgPath(), (topicName, m) -> {
            brakesDown = ((BrakeStateMessage) m).isDown();
        });

        new Subscriber(NodeChannel.BATTERY.getMsgPath(), (topicName, m) -> {
            batteryLevel = ((BatteryLevelMessage) m).getBatteryLevel();
        });


        new Subscriber(NodeChannel.IMU_ANG_POS.getMsgPath(), ((topicName, m) -> {
            IMUAngularPositionMessage mes = ((IMUAngularPositionMessage) m);
            double y = mes.getRot()[0][1];
            double x = mes.getRot()[0][0];
            
            imuAngle = -Math.atan2(y, x);
        }));
        
        new Subscriber(NodeChannel.GPS.getMsgPath(), ((topicName, m) -> {
            if (prevGPS == null) {
                prevGPS = ((GpsMeasurement) m);
            }
            else {
                GpsMeasurement gps = ((GpsMeasurement) m);
                double dlat = gps.getLatitude() - prevGPS.getLatitude();
                double dlon = gps.getLongitude() - prevGPS.getLongitude();

                gpsAngle = -Math.atan2(dlat, dlon);
            }
        }));
    }

    private static BufferedImage toBufferedImage(Image img)
    {
        if (img instanceof BufferedImage)
        {
            return (BufferedImage) img;
        }

        // Create a buffered image with transparency
        BufferedImage bimage = new BufferedImage(img.getWidth(null), img.getHeight(null), BufferedImage.TYPE_INT_ARGB);

        // Draw the image on to the buffered image
        Graphics2D bGr = bimage.createGraphics();
        bGr.drawImage(img, 0, 0, null);
        bGr.dispose();

        // Return the buffered image
        return bimage;
    }

    @Override
    protected void paintComponent(Graphics f) {
        super.paintComponent(f);

        int brakeX = 50;
        int brakeY = 0;
        String status = "up";

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
        g.drawString("batt = " + batteryLevel, brakeX + getHeight()/3 + 10, getHeight()/2);
        g.drawString("status = " + status, brakeX, getHeight()/2);

        // shamelessly ripped from stackoverflow
        g.drawImage(imuArrow, new AffineTransform(Math.cos(imuAngle), Math.sin(imuAngle), -Math.sin(imuAngle), Math.cos(imuAngle), 500, 200), null);
        g.drawImage(gpsArrow, new AffineTransform(Math.cos(gpsAngle), Math.sin(gpsAngle), -Math.sin(gpsAngle), Math.cos(gpsAngle), 700, 100), null);

    }
}
