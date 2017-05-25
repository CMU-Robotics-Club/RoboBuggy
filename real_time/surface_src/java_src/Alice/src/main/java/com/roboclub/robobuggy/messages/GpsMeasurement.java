package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;

import java.util.Date;

/**
 * Message used to pass GPS measurements over BuggyROS
 * gpsTimestamp is the UTC time in hours for the given position.
 * See http://hemispheregnss.com/gpsreference/GPGGA.html for more information
 *
 * @author Matt Sebek (msebek)
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: TODO
 */
public class GpsMeasurement extends BaseMessage {
    public static final String VERSION_ID = "gpsV0.2";

    private Date gpsTimestamp;
    private double latitude;
    private boolean north;
    private double longitude;
    private boolean west;
    private int qualityValue;
    private int numSatellites;
    private double horizontalDilutionOfPrecision;
    private double antennaAltitude;
    private double rawGPSLat;
    private double rawGPSLong;

    /**
     * Constructs a new {@link GpsMeasurement} at time now
     *
     * @param gpsTimestamp                  {@link Date} of the GPS timestamp
     * @param latitude                      latitude measurement
     * @param north                         true iff the latitude measurement is north
     * @param longitude                     longitude measurement
     * @param west                          true iff the longitude measurement is west
     * @param qualityValue                  value of the signal quality
     * @param numSatellites                 number of GPS satellites connected
     * @param horizontalDilutionOfPrecision horizontal dilation of precision
     * @param antennaAltitude               altitude of the antenna
     * @param rawGPSLat                     raw GPS latitude
     * @param rawGPSLong                    raw GPS longitude
     */
    public GpsMeasurement(Date gpsTimestamp, double latitude, boolean north, double longitude,
                          boolean west, int qualityValue, int numSatellites,
                          double horizontalDilutionOfPrecision, double antennaAltitude, double rawGPSLat, double rawGPSLong) {
        this.timestamp = new Date().getTime();
        this.gpsTimestamp = new Date(gpsTimestamp.getTime());
        this.latitude = latitude;
        this.north = north;
        this.longitude = longitude;
        this.west = west;
        this.qualityValue = qualityValue;
        this.numSatellites = numSatellites;
        this.horizontalDilutionOfPrecision = horizontalDilutionOfPrecision;
        this.antennaAltitude = antennaAltitude;
        this.rawGPSLat = rawGPSLat;
        this.rawGPSLong = rawGPSLong;
    }

    /**
     * Constructs a new {@link GpsMeasurement}
     *
     * @param messageTimestamp              {@link Date} representing the time of the message
     * @param gpsTimestamp                  {@link Date} of the GPS timestamp
     * @param latitude                      latitude measurement
     * @param north                         true iff the latitude measurement is north
     * @param longitude                     longitude measurement
     * @param west                          true iff the longitude measurement is west
     * @param qualityValue                  value of the signal quality
     * @param numSatellites                 number of GPS satellites connected
     * @param horizontalDilutionOfPrecision horizontal dilation of precision
     * @param antennaAltitude               altitude of the antenna
     */
    public GpsMeasurement(Date messageTimestamp, Date gpsTimestamp, double latitude, boolean north, double longitude,
                          boolean west, int qualityValue, int numSatellites, double horizontalDilutionOfPrecision, double antennaAltitude) {
        this.timestamp = new Date(messageTimestamp.getTime()).getTime();
        this.gpsTimestamp = new Date(gpsTimestamp.getTime());
        this.latitude = latitude;
        this.north = north;
        this.longitude = longitude;
        this.west = west;
        this.qualityValue = qualityValue;
        this.numSatellites = numSatellites;
        this.horizontalDilutionOfPrecision = horizontalDilutionOfPrecision;
        this.antennaAltitude = antennaAltitude;
    }

    /**
     * A constructor for a gps measurement that is as basic as possible
     *
     * @param latitude  latitude measurement
     * @param longitude longitude measurement
     */
    public GpsMeasurement(double latitude, double longitude) {
        this.timestamp = new Date().getTime();
        this.gpsTimestamp = new Date();
        this.latitude = latitude;
        this.north = latitude > 0;
        this.longitude = longitude;
        this.west = longitude < 0;
        this.qualityValue = 0;
        this.numSatellites = -1;
        this.horizontalDilutionOfPrecision = 0.0;
        this.antennaAltitude = 0.0;
    }

    /**
     * Returns the latitude of the {@link GpsMeasurement}
     *
     * @return the latitude of the {@link GpsMeasurement}
     */
    public double getLatitude() {
        return latitude;
    }

    /**
     * Returns the longitude of the {@link GpsMeasurement}
     *
     * @return the longitude of the {@link GpsMeasurement}
     */
    public double getLongitude() {
        return longitude;
    }

    /**
     * Returns the west value of the {@link GpsMeasurement}
     *
     * @return the west value of the {@link GpsMeasurement}
     */
    public boolean getWest() {
        return west;
    }

    /**
     * Returns the north value of the {@link GpsMeasurement}
     *
     * @return the north value of the {@link GpsMeasurement}
     */
    public boolean getNorth() {
        return north;
    }

    public int getNumSatellites() {return numSatellites; }

    public double getHorizontalDilutionOfPosition() {return horizontalDilutionOfPrecision; }

    public int getQualityValue() {return qualityValue; }

    public double getAntennaAltitude() { return antennaAltitude; }

    /**
     * evaluates to a GPSPoseMessage with the same values encoded in this GPSMeasurement
     *
     * @param heading the heading you want for the gps measurement
     * @return the gps pose message encoding of this message
     */
    public GPSPoseMessage toGpsPoseMessage(double heading) {
        return new GPSPoseMessage(gpsTimestamp, latitude, longitude, heading);
    }
}