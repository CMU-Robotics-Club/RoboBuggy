package com.roboclub.robobuggy.map;

/**
 * Class used to represent a rectangle in a {@link Map}
 *
 * @author Trevor Decker
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: TODO
 * @deprecated use polygon
 */
public class Rect implements MapObject {
    private Point uR;
    private Point uL;
    private Point lL;

    /**
     * Constructs a new {@link Rect} object
     *
     * @param uR {@link Point} representing the upper right hand corner
     * @param uL {@link Point} representing the upper left hand corner
     * @param lL {@link Point} representing the lower left hand corner
     */
    public Rect(Point uR, Point uL, Point lL) {
        this.uR = uR;
        this.uL = uL;
        this.lL = lL;
    }

    /**
     * Determines if a {@link Point} is inside the {@link Rect}
     *
     * @param marker {@link Point} to check
     * @return true iff the {@link Point} is within the {@link Rect}
     */
    public boolean within(Point marker) {
        if (marker != null) {
            return (marker.getX() >= this.uL.getX())
                    && (marker.getX() <= this.uR.getX())
                    && (marker.getY() >= this.lL.getY())
                    && (marker.getY() <= this.uL.getY());
        }

        return false;
    }
}
