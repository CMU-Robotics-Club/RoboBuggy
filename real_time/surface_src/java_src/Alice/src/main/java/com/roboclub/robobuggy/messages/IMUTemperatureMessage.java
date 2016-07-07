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

    private double temperature;

    /**
     * Constructs a new {@link IMUTemperatureMessage} at time now
     *
     * @param temp temperature reading from the IMU
     */
    public IMUTemperatureMessage(double temp) {
        this.timestamp = new Date().getTime();
        this.temperature = temp;
    }

    /**
     * Returns the temperature value of the {@link IMUTemperatureMessage}
     *
     * @return the temperature value of the {@link IMUTemperatureMessage}
     */
    public double getTemp() {
        return this.temperature;
    }

}
