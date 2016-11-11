package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU temperature measurements within BuggyROS
 *
 * @author ?
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: TODO
 */
public class IMUTemperatureMessage extends BaseMessage {

    public static final String VERSION_ID = "temperatureV0.1";

    private int IMU;
    private double temperature;

    /**
     * Constructs a new {@link IMUTemperatureMessage} at time now with IMU ID = 0
     *
     * @param temp temperature reading from the IMU
     */
    public IMUTemperatureMessage(double temp) {
        this.timestamp = new Date().getTime();
        this.temperature = temp;
        this.IMU = 0;
    }

    /**
     * Constructs a new {@link IMUTemperatureMessage} at time now
     *
     * @param temp temperature reading from the IMU
     * @param imuID the ID of the IMU that generated the data
     */
    public IMUTemperatureMessage(double temp, int imuID) {
        this.timestamp = new Date().getTime();
        this.temperature = temp;
        this.IMU = imuID;
    }

    /**
     * Returns the temperature value of the {@link IMUTemperatureMessage}
     *
     * @return the temperature value of the {@link IMUTemperatureMessage}
     */
    public double getTemp() {
        return this.temperature;
    }

    /**
     * Returns the ID of the IMU for the {@link IMUTemperatureMessage}
     *
     * @return the ID of the IMU for the {@link IMUTemperatureMessage}
     */
    public int getIMU() { return this.IMU; }

}
