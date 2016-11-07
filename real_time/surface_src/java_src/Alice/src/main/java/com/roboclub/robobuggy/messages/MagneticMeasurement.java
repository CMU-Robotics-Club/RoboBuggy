package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU magnetic north message within BuggyROS
 *
 * @author ?
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: TODO
 */
public class MagneticMeasurement extends BaseMessage {

    public static final String VERSION_ID = "magV0.0";

    private int IMU;

    private double magX;
    private double magY;
    private double magZ;

    /**
     * Constructs a new {@link MagMeasurement} at time now
     *
     * @param magX magX value
     * @param magY magY value
     * @param magZ magZ value
     */
    public MagneticMeasurement(double magX, double magY, double magZ, int imuID) {
        this.timestamp = new Date().getTime();
        this.magX = magX;
        this.magY = magY;
        this.magZ = magZ;
        this.IMU = imuID;
    }

    /**
     * Returns the magX value of the {@link MagneticMeasurement}
     *
     * @return the magX value of the {@link MagneticMeasurement}
     */
    public double getmagX() {
        return magX;
    }

    /**
     * Returns the magY value of the {@link MagneticMeasurement}
     *
     * @return the magY value of the {@link MagneticMeasurement}
     */
    public double getmagY() {
        return magY;
    }

    /**
     * Returns the magZ value of the {@link MagneticMeasurement}
     *
     * @return the magZ value of the {@link MagneticMeasurement}
     */
    public double getmagZ() {
        return magZ;
    }


    /**
     * Returns the ID of the IMU for the {@link MagneticMeasurement}
     *
     * @return the ID of the IMU for the {@link MagneticMeasurement}
     */
    public int getIMU() { return this.IMU; }
}
