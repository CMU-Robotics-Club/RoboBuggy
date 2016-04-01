package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.ui.LocTuple;

import java.util.Map;

/**
 * Created by vivaanbahl on 3/31/16.
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
     * @param meters the distance travelled in meters
     * @param heading the heading travelled <b>(IN DEGREES)</b>
     * @return a {@link LocTuple} whose lat and lon are the DELTAS to add to a position estimate
     */
    public static LocTuple convertMetersToLatLng(double meters, double heading) {

        double deltaMetersX = meters * Math.cos(Math.toRadians(heading));
        double deltaMetersY = meters * Math.sin(Math.toRadians(heading));

        double deltaDegreesLat = deltaMetersY/ONE_DEG_LAT_TO_METERS;
        double deltaDegreesLon = deltaMetersX/ONE_DEG_LON_TO_METERS_PITTSBURGH;

        return new LocTuple(deltaDegreesLat, deltaDegreesLon);
    }

}
