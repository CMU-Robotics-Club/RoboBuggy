package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.ui.LocTuple;
import com.sun.tools.javac.util.Pair;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Tests the Localizer util functions
 */
public class LocalizerUtilTest {

    private final double latPittsburgh = 40.440310;
    private final double lonPittsburgh = -79.9471537;
    private final double calcedDistOneDegMeters = 84721.0;

    @Test
    public void test_convertLatLngDeltaToMeters() throws Exception {

        LocTuple pitt = new LocTuple(latPittsburgh, lonPittsburgh);
        LocTuple pittplusone = new LocTuple(latPittsburgh, lonPittsburgh + 1);

        Pair<Double, Double> deltaAndHeading = LocalizerUtil.convertLatLngDeltaToMeters(pitt, pittplusone);

        assertEquals(deltaAndHeading.fst, calcedDistOneDegMeters, 100);
        assertEquals(deltaAndHeading.snd, 0.0, 0.0000001);

    }

    @Test
    public void test_convertMetersToLatLng() throws Exception {

        LocTuple deltaPos = LocalizerUtil.convertMetersToLatLng(calcedDistOneDegMeters, 0.0);

        assertEquals(deltaPos.getLatitude(), 0.0, 0.00001);
        assertEquals(deltaPos.getLongitude(), 1.0, 0.00005);

    }

}