package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.io.UnsupportedEncodingException;

/**
 * {@link SerialNode} for reading in IMU data
 *
 * @author Matt Sebek
 * @author Kevin Brennan
 * @author Trevor Decker
 */
public final class ImuNode extends SerialNode {
    /**
     * Baud rate for serial port
     */
    private static final int BAUDRATE = 57600;
    // how long the system should wait until a sensor switches to Disconnected
    private static final long SENSOR_TIME_OUT = 5000;

    private Publisher msgPubIMU;
    private Publisher msgPubMAG;

    private Publisher statePub;

    /**
     * Creates a new {@link ImuNode}
     *
     * @param sensor   {@link NodeChannel} of IMU
     * @param portName name of the serial port to read from
     */
    public ImuNode(NodeChannel sensor, String portName) {
        super(new BuggyBaseNode(sensor), "IMU", portName, BAUDRATE);
        msgPubIMU = new Publisher(NodeChannel.IMU.getMsgPath());
        msgPubMAG = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());
        statePub = new Publisher(sensor.getStatePath());


        // TODO state stuff
        statePub.publish(new StateMessage(NodeState.DISCONNECTED));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean matchDataSample(byte[] sample) {
        return true;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int matchDataMinSize() {
        return 0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int getBaudRate() {
        return 57600;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int peel(byte[] buffer, int start, int bytesAvailable) {
        // TODO replace 80 with max message length
        if (bytesAvailable < 50) {
            // Not enough bytes...maybe?
            return 0;
        }

        // Check the prefix. was previously #ACG
        if (buffer[start] != '#') {
            return 1;
        }
        if (buffer[start + 1] != 'Y') {
            return 1;
        }
        if (buffer[start + 2] != 'P') {
            return 1;
        }
        if (buffer[start + 3] != 'R') {
            return 1;
        }
        if (buffer[start + 4] != '=') {
            return 1;
        }

        double[] vals = new double[6];
        String imuRawStr;
        try {
            imuRawStr = new String(buffer, start + 5, bytesAvailable - 5, "UTF-8");

        } catch (UnsupportedEncodingException e) {
            return 1;
        } //TODO check +5 -5

        int origLength = imuRawStr.length();
        for (int i = 0; i < 5; i++) {
            // TODO: need less than bytes_availble
            int commaIndex = imuRawStr.indexOf(',');
            try {
                Double d = Double.parseDouble(imuRawStr.substring(0, commaIndex));
                vals[i] = d;
            } catch (NumberFormatException nfe) {
                return 1;
            }
            imuRawStr = imuRawStr.substring(commaIndex + 1);
        }


        // The last one, we use the hash as the symbol!
        int hashIndex = imuRawStr.indexOf('#');
        if (hashIndex == -1) {
            return 1;
        }
        try {
            vals[5] = Double.parseDouble(imuRawStr.substring(0, hashIndex));
        } catch (NumberFormatException e) {
            e.printStackTrace();
        }
        imuRawStr = imuRawStr.substring(hashIndex);

        msgPubIMU.publish(new ImuMeasurement(vals[0], vals[1], vals[2]));
        msgPubMAG.publish(new MagneticMeasurement(vals[3], vals[4], vals[5]));
        //Feed the watchdog
        setNodeState(NodeState.ON);
        return 4 + (origLength - imuRawStr.length());

    }

}