package com.roboclub.robobuggy.nodes.localizers;

import java.util.Map;

/**
 * Created by vivaanbahl on 3/31/16.
 * A location to store helper functions for localizers
 */
public class LocalizerUtil {

    // some math for the conversion of latitude to meters
    // obtained from Wikipedia article on latitude as well as Google Maps
    private static final double ONE_DEG_LON_TO_METERS_EQUATOR = 111319.9;
    private static final double ONE_DEG_LAT_TO_METERS = ONE_DEG_LON_TO_METERS_EQUATOR; // delta Lat doesn't depend on degree
    private static final double LAT_DEGREE_PITTSBURGH = 40.440310; // center of Schenley
    private static final double ONE_DEG_LON_TO_METERS_PITTSBURGH =
            ONE_DEG_LON_TO_METERS_EQUATOR * Math.cos(Math.toRadians(LAT_DEGREE_PITTSBURGH));

    /**
     * Takes 2 GPS points and computes the distance and heading between them
     *
     * @param point1 point 1
     * @param point2 point 2
     * @return a pair whose 1st is the distance in meters, and whose 2nd is the heading in radians
     */
    public static Map.Entry<Double, Double> convertLatLngDeltaToMeters(LocTuple point1, LocTuple point2) {

        double deltaLon = point2.getLongitude() - point1.getLongitude();
        double deltaLat = point2.getLatitude() - point1.getLatitude();

        double deltaMetersX = deltaLon * ONE_DEG_LON_TO_METERS_PITTSBURGH;
        double deltaMetersY = deltaLat * ONE_DEG_LAT_TO_METERS;

        double deltaMeters = Math.sqrt(deltaMetersX * deltaMetersX + deltaMetersY * deltaMetersY);
        double heading = Math.atan2(deltaLat, deltaLon);

        return new Map.Entry<Double, Double>() {
            @Override
            public Double getKey() {
                return deltaMeters;
            }

            @Override
            public Double getValue() {
                return heading;
            }

            @Override
            public Double setValue(Double value) {
                return null;
            }

        };
    }

    /**
     * Converts a delta distance into a delta of GPS coordinates
     *
     * @param meters  the distance travelled in meters
     * @param heading the heading travelled <b>(IN DEGREES)</b>
     * @return a {@link LocTuple} whose lat and lon are the DELTAS to add to a position estimate
     */
    public static LocTuple convertMetersToLatLng(double meters, double heading) {

        double deltaMetersX = meters * Math.cos(Math.toRadians(heading));
        double deltaMetersY = meters * Math.sin(Math.toRadians(heading));


        double deltaDegreesLat = convertMetersToLat(deltaMetersY);
        double deltaDegreesLon = convertMetersToLon(deltaMetersX);

        return new LocTuple(deltaDegreesLat, deltaDegreesLon);
    }

    /**
     * converts meters to Lattitude
     *
     * @param meters distince in meters
     * @return distince in lattitude
     */
    public static double convertMetersToLat(double meters) {
        return meters / ONE_DEG_LAT_TO_METERS;
    }

    /**
     * converts meters to longitude only works around pittsbrugh
     *
     * @param meters distance in meters
     * @return distance in lattitude
     */
    public static double convertMetersToLon(double meters) {
        return meters / ONE_DEG_LON_TO_METERS_PITTSBURGH;
    }

    /**
     * Converts latitude to meters
     *
     * @param lat distance in latitude
     * @return distance in meters
     */
    public static double convertLatToMeters(double lat) {
        return lat * ONE_DEG_LAT_TO_METERS;
    }

    /**
     * converts longitude to meters
     *
     * @param lon distance in longitude
     * @return distance in meters
     */
    public static double convertLonToMeters(double lon) {
        return lon * ONE_DEG_LON_TO_METERS_PITTSBURGH;
    }

    // 

    /**
     * UTM (meter-based projections) to GPS Lat/Long is from
     * http://stackoverflow.com/questions/176137/java-convert-lat-lon-to-utm
     * It looks like it is based on the same algorithm as
     * http://www.rcn.montana.edu/resources/converter.aspx
     *
     * @param loc a gps location tuple to be converted to a utm tuple
     * @return utm tuple of the position
     */
    public static UTMTuple deg2UTM(LocTuple loc) {
        double lat = loc.getLatitude();
        double lon = loc.getLongitude();
        double easting;
        double northing;
        int zone;
        char letter;
        zone = (int) Math.floor(lon / 6 + 31);
        if (lat < -72)
            letter = 'C';
        else if (lat < -64)
            letter = 'D';
        else if (lat < -56)
            letter = 'E';
        else if (lat < -48)
            letter = 'F';
        else if (lat < -40)
            letter = 'G';
        else if (lat < -32)
            letter = 'H';
        else if (lat < -24)
            letter = 'J';
        else if (lat < -16)
            letter = 'K';
        else if (lat < -8)
            letter = 'L';
        else if (lat < 0)
            letter = 'M';
        else if (lat < 8)
            letter = 'N';
        else if (lat < 16)
            letter = 'P';
        else if (lat < 24)
            letter = 'Q';
        else if (lat < 32)
            letter = 'R';
        else if (lat < 40)
            letter = 'S';
        else if (lat < 48)
            letter = 'T';
        else if (lat < 56)
            letter = 'U';
        else if (lat < 64)
            letter = 'V';
        else if (lat < 72)
            letter = 'W';
        else
            letter = 'X';
        easting = 0.5 * Math.log((1 + Math.cos(lat * Math.PI / 180) * Math.sin(lon * Math.PI / 180 - (6 * zone - 183) * Math.PI / 180))
                / (1 - Math.cos(lat * Math.PI / 180) * Math.sin(lon * Math.PI / 180 - (6 * zone - 183) * Math.PI / 180)))
                * 0.9996 * 6399593.62 / Math.pow((1 + Math.pow(0.0820944379, 2) * Math.pow(Math.cos(lat * Math.PI / 180), 2)), 0.5)
                * (1 + Math.pow(0.0820944379, 2) / 2 * Math.pow((0.5 * Math.log((1 + Math.cos(lat * Math.PI / 180)
                * Math.sin(lon * Math.PI / 180 - (6 * zone - 183) * Math.PI / 180)) / (1 - Math.cos(lat * Math.PI / 180)
                * Math.sin(lon * Math.PI / 180 - (6 * zone - 183) * Math.PI / 180)))), 2) * Math.pow(Math.cos(lat * Math.PI / 180), 2) / 3) + 500000;

        easting = Math.round(easting * 100) * 0.01;

        northing = (Math.atan(Math.tan(lat * Math.PI / 180) / Math.cos((lon * Math.PI / 180 - (6 * zone - 183) * Math.PI / 180))) - lat * Math.PI / 180)
                * 0.9996 * 6399593.625 / Math.sqrt(1 + 0.006739496742 * Math.pow(Math.cos(lat * Math.PI / 180), 2)) * (1 + 0.006739496742 / 2 *
                Math.pow(0.5 * Math.log((1 + Math.cos(lat * Math.PI / 180) * Math.sin((lon * Math.PI / 180 - (6 * zone - 183) * Math.PI / 180)))
                        / (1 - Math.cos(lat * Math.PI / 180) * Math.sin((lon * Math.PI / 180 - (6 * zone - 183) * Math.PI / 180)))), 2) *
                Math.pow(Math.cos(lat * Math.PI / 180), 2)) + 0.9996 * 6399593.625 * (lat * Math.PI / 180 - 0.005054622556 *
                (lat * Math.PI / 180 + Math.sin(2 * lat * Math.PI / 180) / 2) + 4.258201531e-05 *
                (3 * (lat * Math.PI / 180 + Math.sin(2 * lat * Math.PI / 180) / 2) + Math.sin(2 * lat * Math.PI / 180) *
                        Math.pow(Math.cos(lat * Math.PI / 180), 2)) / 4 - 1.674057895e-07 *
                (5 * (3 * (lat * Math.PI / 180 + Math.sin(2 * lat * Math.PI / 180) / 2) + Math.sin(2 * lat * Math.PI / 180)
                        * Math.pow(Math.cos(lat * Math.PI / 180), 2)) / 4 + Math.sin(2 * lat * Math.PI / 180)
                        * Math.pow(Math.cos(lat * Math.PI / 180), 2) * Math.pow(Math.cos(lat * Math.PI / 180), 2)) / 3);
        if (letter < 'M')
            northing = northing + 10000000;
        northing = Math.round(northing * 100) * 0.01;

        return new UTMTuple(zone, letter, easting, northing);
    }

    /**
     * Converts from utm to gps degrees
     *
     * @param loc the input utm tuple
     * @return the output gps coordinates
     */
    public static LocTuple utm2Deg(UTMTuple loc) {
        int zone = loc.getZone();
        char letter = loc.getLetter();
        double easting = loc.getEasting();
        double northing = loc.getNorthing();
        double latitude;
        double longitude;
        double hem;
        if (letter > 'M')
            hem = 'N';
        else
            hem = 'S';
        double north;
        if (hem == 'S')
            north = northing - 10000000;
        else
            north = northing;
        latitude = (north / 6366197.724 / 0.9996 + (1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996)
                , 2) - 0.006739496742 * Math.sin(north / 6366197.724 / 0.9996) * Math.cos(north / 6366197.724 / 0.9996)
                * (Math.atan(Math.cos(Math.atan((Math.exp((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 +
                0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))) * (1 - 0.006739496742 *
                Math.pow((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(
                        Math.cos(north / 6366197.724 / 0.9996), 2)))), 2) / 2 * Math.pow(Math.cos(north / 6366197.724
                / 0.9996), 2) / 3)) - Math.exp(-(easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((
                1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))) * (1 - 0.006739496742 *
                Math.pow((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(
                        Math.cos(north / 6366197.724 / 0.9996), 2)))), 2) / 2 * Math.pow(Math.cos(north / 6366197.724 /
                0.9996), 2) / 3))) / 2 / Math.cos((north - 0.9996 * 6399593.625 * (north / 6366197.724 / 0.9996 -
                0.006739496742 * 3 / 4 * (north / 6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) /
                        2) + Math.pow(0.006739496742 * 3 / 4, 2) * 5 / 3 * (3 * (north / 6366197.724 / 0.9996 +
                Math.sin(2 * north / 6366197.724 / 0.9996) / 2) + Math.sin(2 * north / 6366197.724 / 0.9996) *
                Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) / 4 - Math.pow(0.006739496742 * 3 / 4, 3) * 35 / 27
                * (5 * (3 * (north / 6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) / 2) +
                Math.sin(2 * north / 6366197.724 / 0.9996) * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) / 4 +
                Math.sin(2 * north / 6366197.724 / 0.9996) * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2) *
                        Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) / 3)) / (0.9996 * 6399593.625 / Math.sqrt(
                (1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))) * (1 - 0.006739496742 *
                Math.pow((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(
                        Math.cos(north / 6366197.724 / 0.9996), 2)))), 2) / 2 * Math.pow(Math.cos(north / 6366197.724
                / 0.9996), 2)) + north / 6366197.724 / 0.9996))) * Math.tan((north - 0.9996 * 6399593.625 * (north /
                6366197.724 / 0.9996 - 0.006739496742 * 3 / 4 * (north / 6366197.724 / 0.9996 + Math.sin(2 * north /
                6366197.724 / 0.9996) / 2) + Math.pow(0.006739496742 * 3 / 4, 2) * 5 / 3 * (3 * (north / 6366197.724 /
                0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) / 2) + Math.sin(2 * north / 6366197.724 / 0.9996) *
                Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) / 4 - Math.pow(0.006739496742 * 3 / 4, 3) * 35 /
                27 * (5 * (3 * (north / 6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) / 2) +
                Math.sin(2 * north / 6366197.724 / 0.9996) * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) / 4 +
                Math.sin(2 * north / 6366197.724 / 0.9996) * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2) *
                        Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) / 3)) / (0.9996 * 6399593.625 /
                Math.sqrt((1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))) * (1 -
                0.006739496742 * Math.pow((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742
                        * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))), 2) / 2 * Math.pow(Math.cos(north /
                        6366197.724 / 0.9996), 2)) + north / 6366197.724 / 0.9996)) - north / 6366197.724 / 0.9996) * 3
                / 2) * (Math.atan(Math.cos(Math.atan((Math.exp((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt(
                (1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))) * (1 - 0.006739496742 *
                Math.pow((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 *
                        Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))), 2) / 2 * Math.pow(Math.cos(north /
                6366197.724 / 0.9996), 2) / 3)) - Math.exp(-(easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 +
                0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))) * (1 - 0.006739496742 *
                Math.pow((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(Math.cos(
                        north / 6366197.724 / 0.9996), 2)))), 2) / 2 * Math.pow(Math.cos(north / 6366197.724 / 0.9996),
                2) / 3))) / 2 / Math.cos((north - 0.9996 * 6399593.625 * (north / 6366197.724 / 0.9996 -
                0.006739496742 * 3 / 4 * (north / 6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) / 2)
                + Math.pow(0.006739496742 * 3 / 4, 2) * 5 / 3 * (3 * (north / 6366197.724 / 0.9996 + Math.sin(2 * north
                / 6366197.724 / 0.9996) / 2) + Math.sin(2 * north / 6366197.724 / 0.9996) * Math.pow(Math.cos(north /
                6366197.724 / 0.9996), 2)) / 4 - Math.pow(0.006739496742 * 3 / 4, 3) * 35 / 27 * (5 * (3 * (north /
                6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) / 2) + Math.sin(2 * north /
                6366197.724 / 0.9996) * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) / 4 + Math.sin(2 * north /
                6366197.724 / 0.9996) * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2) * Math.pow(Math.cos(north /
                6366197.724 / 0.9996), 2)) / 3)) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(
                Math.cos(north / 6366197.724 / 0.9996), 2)))) * (1 - 0.006739496742 * Math.pow((easting - 500000) /
                (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996)
                        , 2)))), 2) / 2 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) + north / 6366197.724 /
                0.9996))) * Math.tan((north - 0.9996 * 6399593.625 * (north / 6366197.724 / 0.9996 - 0.006739496742 * 3
                / 4 * (north / 6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) / 2) + Math.pow(
                0.006739496742 * 3 / 4, 2) * 5 / 3 * (3 * (north / 6366197.724 / 0.9996 + Math.sin(2 * north /
                6366197.724 / 0.9996) / 2) + Math.sin(2 * north / 6366197.724 / 0.9996) * Math.pow(Math.cos(north /
                6366197.724 / 0.9996), 2)) / 4 - Math.pow(0.006739496742 * 3 / 4, 3) * 35 / 27 * (5 * (3 * (north /
                6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) / 2) + Math.sin(2 * north /
                6366197.724 / 0.9996) * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) / 4 + Math.sin(2 * north
                / 6366197.724 / 0.9996) * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2) * Math.pow(Math.cos(north
                / 6366197.724 / 0.9996), 2)) / 3)) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 *
                Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))) * (1 - 0.006739496742 * Math.pow((easting -
                500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724
                / 0.9996), 2)))), 2) / 2 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2))
                + north / 6366197.724 / 0.9996)) - north / 6366197.724 / 0.9996)) * 180 / Math.PI;

        //TODO clean this up
        latitude = Math.round(latitude * 10000000);
        latitude = latitude / 10000000;
        longitude = Math.atan((Math.exp((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 *
                Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))) * (1 - 0.006739496742 * Math.pow((easting -
                500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724
                / 0.9996), 2)))), 2) / 2 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2) / 3)) - Math.exp(
                -(easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(Math.cos(north
                        / 6366197.724 / 0.9996), 2)))) * (1 - 0.006739496742 * Math.pow((easting - 500000) / (0.9996 *
                        6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996),
                        2)))), 2) / 2 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2) / 3))) / 2 / Math.cos(
                (north - 0.9996 * 6399593.625 * (north / 6366197.724 / 0.9996 - 0.006739496742 * 3 / 4 * (north /
                        6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) / 2) + Math.pow(0.006739496742
                        * 3 / 4, 2) * 5 / 3 * (3 * (north / 6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 /
                        0.9996) / 2) + Math.sin(2 * north / 6366197.724 / 0.9996) * Math.pow(Math.cos(north /
                        6366197.724 / 0.9996), 2)) / 4 - Math.pow(0.006739496742 * 3 / 4, 3) * 35 / 27 * (5 * (3 *
                        (north / 6366197.724 / 0.9996 + Math.sin(2 * north / 6366197.724 / 0.9996) / 2) +
                        Math.sin(2 * north / 6366197.724 / 0.9996) * Math.pow(Math.cos(north / 6366197.724 / 0.9996),
                                2)) / 4 + Math.sin(2 * north / 6366197.724 / 0.9996) * Math.pow(Math.cos(north /
                        6366197.724 / 0.9996), 2) * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)) / 3)) / (0.9996
                        * 6399593.625 / Math.sqrt((1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996),
                        2)))) * (1 - 0.006739496742 * Math.pow((easting - 500000) / (0.9996 * 6399593.625 / Math.sqrt(
                        (1 + 0.006739496742 * Math.pow(Math.cos(north / 6366197.724 / 0.9996), 2)))), 2) / 2 * Math.pow(
                        Math.cos(north / 6366197.724 / 0.9996), 2))
                        + north / 6366197.724 / 0.9996)) * 180 / Math.PI + zone * 6 - 183;

        longitude = Math.round(longitude * 10000000);
        longitude = longitude / 10000000;

        return new LocTuple(latitude, longitude);
    }

}
