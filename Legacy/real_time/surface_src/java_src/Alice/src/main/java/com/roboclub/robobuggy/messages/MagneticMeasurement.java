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
    public MagneticMeasurement(double magX, double magY, double magZ) {
        this.timestamp = new Date().getTime();
        this.magX = magX;
        this.magY = magY;
        this.magZ = magZ;
    }

    /**
     * Returns the magX value of the {@link MagMeasurement}
     *
     * @return the magX value of the {@link MagMeasurement}
     */
    public double getmagX() {
        return magX;
    }

    /**
     * Returns the magY value of the {@link MagMeasurement}
     *
     * @return the magY value of the {@link MagMeasurement}
     */
    public double getmagY() {
        return magY;
    }

    /**
     * Returns the magZ value of the {@link MagMeasurement}
     *
     * @return the magZ value of the {@link MagMeasurement}
     */
    public double getmagZ() {
        return magZ;
    }

}
