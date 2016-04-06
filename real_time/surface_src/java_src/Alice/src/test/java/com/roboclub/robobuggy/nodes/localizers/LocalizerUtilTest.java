package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.ui.LocTuple;
import org.junit.Test;

import java.util.Map;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;
import static org.junit.Assert.assertEquals;

/**
 * Tests the Localizer util functions
 */
public class LocalizerUtilTest {

    private static final double latPittsburgh = 40.440310;
    private static final double lonPittsburgh = -79.9471537;
    private static final double calcedDistOneDegMeters = 84721.0;

    // the distance in meters for 1˚ up and 1˚ right
    private static final double diagDistMeters = 139505.0;

    private static final double FEET_TO_METERS = 0.3048;

    @Test
    public void test_convertLatLngDeltaToMeters() throws Exception {

        LocTuple pitt = new LocTuple(latPittsburgh, lonPittsburgh);
        LocTuple oneLonRight = new LocTuple(latPittsburgh, lonPittsburgh + 1);
        LocTuple diagonalRightAndUp = new LocTuple(latPittsburgh + 1, lonPittsburgh + 1);

        // one to the right
        Map.Entry<Double, Double> deltaAndHeading = LocalizerUtil.convertLatLngDeltaToMeters(pitt, oneLonRight);

        assertEquals(deltaAndHeading.getKey(), calcedDistOneDegMeters, 500);
        assertEquals(deltaAndHeading.getValue(), 0.0, 0.0000001);

        // one right and one up
        deltaAndHeading = LocalizerUtil.convertLatLngDeltaToMeters(pitt, diagonalRightAndUp);

        assertEquals(diagDistMeters, deltaAndHeading.getKey(), 500);
        assertEquals(PI/4, deltaAndHeading.getValue(), 0.01);

    }

    @Test
    public void test_convertMetersToLatLng() throws Exception {

        LocTuple deltaPos = LocalizerUtil.convertMetersToLatLng(calcedDistOneDegMeters, 0.0);

        assertEquals(0.0, deltaPos.getLatitude(), 0.00001);
        assertEquals(1.0, deltaPos.getLongitude(), 0.00005);

        double dx = -1353.14 * FEET_TO_METERS;
        double dy = 413.79 * FEET_TO_METERS;
        double dist = sqrt(dx * dx + dy * dy);
        LocTuple p1 = new LocTuple(40.440444, -79.942140);
        LocTuple p2 = new LocTuple(40.441522, -79.947020);

        double heading = Math.toDegrees(Math.atan2(dy, dx));

        deltaPos = LocalizerUtil.convertMetersToLatLng(dist, heading);

        assertEquals(p2.getLatitude() - p1.getLatitude(), deltaPos.getLatitude(), 0.0002);
        assertEquals(p2.getLongitude() - p1.getLongitude(), deltaPos.getLongitude(), 0.0002);

    }

}