package com.roboclub.robobuggy.map;

/**
 * Representation of a point on a {@link Map}.
 * For now this value will be stored as a 2d (x,y) pair
 */
public class Point implements MapObject {
    private double x;
    private double y;

    /**
     * Constructs a new {@link Point} object
     *
     * @param x x-coordinate of the {@link Point}
     * @param y y-coordinate of the {@link Point}
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the x-coordinate of the {@link Point}
     *
     * @return the x-coordinate of the {@link Point}
     */
    public double getX() {
        return this.x;
    }

    /**
     * Returns the y-coordinate of the {@link Point}
     *
     * @return the y-coordinate of the {@link Point}
     */
    public double getY() {
        return this.y;
    }

    /**
     * Sets the x-coordinate of the {@link Point}
     *
     * @param x x-coordinate
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Sets the y-coordinate of the {@link Point}
     *
     * @param y y-coordinate
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * Calculates the distance to another {@link Point}
     *
     * @param point {@link Point} to get the distance to
     * @return the distance to another {@link Point}
     */
    public double getDistance(Point point) {
        return Math.sqrt(Math.pow((point.x - x), 2) +
                Math.pow((point.y - y), 2));
    }

    /**
     * Calculates the dot product with another {@link Point}
     *
     * @param aPoint {@link Point} to take the dot product with
     * @return the dot product of aPoint and the calling point
     */
    public double dotProduct(Point aPoint) {
        return this.x * aPoint.x + this.y * aPoint.y;
    }
}
