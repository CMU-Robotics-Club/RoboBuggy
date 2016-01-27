package com.roboclub.robobuggy.map;

import java.util.ArrayList;
import java.util.List;

//TODO: write Line test system

/**
 * Object used to represent a line in a {@link Map}
 */
public class Line implements MapObject {
	private List<Point> points = new ArrayList<>();
	private static final double ON_LINE_DISTANCE = .1; // meters
	private boolean validLine = false;

	// if the line is not valid then all operations are not valid
	// must have at least two points

	/**
	 * Construct a new {@link Line} from a {@link List} of {@link Point}s
	 * @param newPoints {@link List} of {@link Point}s to form the 
	 * {@link Line} from
	 */
	public Line(ArrayList<Point> newPoints) {
		for (int i = 0; i < newPoints.size(); i++) {
			points.add(newPoints.get(i));
		}
		updatedValidLine();
	}

	/**
	 * Constructs a new {@link Line} given a start and end {@link Point}
	 * @param aStart {@link Point} to start from
	 * @param aEnd {@link Point} to end at
	 */
	public Line(Point aStart, Point aEnd) {
		points = new ArrayList<>();
		points.add(aStart);
		points.add(aEnd);
		updatedValidLine();
	}

	/**
	 * Constructs a new empty line (containing no {@link Point}s)
	 */
	public Line() {
		points = new ArrayList<Point>();
		updatedValidLine();
	}

	/**
	 * Determines if a {@link Line} is valid (contains 2 or more 
	 * {@link Point}s)
	 */
	private void updatedValidLine() {
		if (points.size() >= 2) {
			validLine = true;
		} else {
			validLine = false;
		}
	}

	/**
	 * Adds a new {@link Point} to the {@link Line}
	 * @param newPoint {@link Point} to add
	 */
	public void addPointToLine(Point newPoint) {
		points.add(newPoint);
		updatedValidLine();
	}

	/**
	 * Removes a {@link Point} from the {@link Line}
	 * @param i index of the {@link Point} to remove
	 */
	public void removePointByIndex(int i) {
		points.remove(i);
		updatedValidLine();
	}

	/**
	 * Removes a {@link Point} from the {@link Line}
	 * @param pointToRemove {@link Point} to remove
	 */
	public void removeEquivlentPoint(Point pointToRemove) {
		for (int i = 0; i < points.size(); i++) {
			if (points.get(i).equals(pointToRemove)) {
				points.remove(i);
				i--; // decrement point since we removed it
			}
		}
		updatedValidLine();
	}

	/**
	 * Determines if a {@link Point} is on the {@link Line}
	 * @param aPoint {@link Point} to check
	 * @return true iff aPoint is on the line
	 */
	public boolean onLine(Point aPoint) {
		assert (validLine);
		return getDistance(aPoint) < ON_LINE_DISTANCE;
	}

	/**
	 * Determines if two {@link Line}s intersect
	 * @param aLine {@link Line} to check
	 * @return true iff the two lines intersect
	 */
	public boolean intersect(Line aLine) {
		assert (validLine && aLine.validLine);
		// TODO
		return false;
	}

	// TODO abstract to work in higher Dimensions
	private boolean lineSegmentsIntersect(Point aStart, Point aEnd,
			Point bStart, Point bEnd) {
		assert (validLine);
		double aSlope = (aEnd.getY() - aStart.getY())
				/ (aEnd.getX() - aStart.getX());
		double bSlope = (bEnd.getY() - bStart.getY())
				/ (bEnd.getX() - bStart.getX());
		double aOffset = aStart.getY() - aStart.getX() * aSlope;
		double bOffset = bStart.getY() - bStart.getX() * bSlope;

		double xIntersect = (bOffset - aOffset) / (aSlope - bSlope);
		double yIntersect = aSlope * xIntersect + aOffset;
		Line aLine = new Line(aStart, aEnd);
		return aLine.onLine(new Point(xIntersect, yIntersect));
	}

	// returns the distance from a point to the line
	// returns in meters
	/**
	 * Calculates the distance from a {@link Point} to the nearest
	 *  {@link Point} contained in the {@link Line}
	 * @param aPoint {@link Point} to check
	 * @return the distance to aPoint
	 */
	public double getDistance(Point aPoint) {
		assert (validLine);
		double minD = Double.MAX_VALUE;
		for (int i = 0; i < points.size() - 1; i++) {
			Point v = new Point(
					points.get(i).getY() - points.get(i + 1).getY(), points
							.get(i + 1).getX() - points.get(i).getX());
			Point r = new Point(points.get(i + 1).getX() - aPoint.getX(),
					points.get(i + 1).getY() - aPoint.getY());
			double d = v.dotProduct(r);
			minD = Math.min(minD, d);
		}
		return minD;
	}

	/**
	 * Combines lineA and lineB into 1 continuous {@link Line}
	 * @param lineA one {@link Line} to combine
	 * @param lineB second {@link Line} to combine
	 * @return a new {@link Line} that combines the two
	 */
	public static Line combineLine(Line lineA, Line lineB) {
		assert (lineA.validLine);
		assert (lineB.validLine);
		Line newLine = new Line();
		for (int i = 0; i < lineA.points.size(); i++) {
			newLine.addPointToLine(lineA.points.get(i));
		}
		for (int i = 0; i < lineB.points.size(); i++) {
			newLine.addPointToLine(lineB.points.get(i));
		}
		return newLine;
	}

	/**
	 * Returns true iff thisLine is a subsection of the {@link Line}
	 * @param thisLine {@link Line} to compare to
	 * @return true iff thisLine is a subsection of the {@link Line}
	 */
	public boolean isSubSection(Line thisLine) {
		assert (validLine && thisLine.validLine);
		for (int i = 0; i < thisLine.points.size(); i++) {
			if (!onLine(thisLine.points.get(i))) {
				return false;
			}
		}
		return true;
	}
	
	/**{@inheritDoc}*/
	@Override
	public boolean equals(Object thisObject) {
		if (!(thisObject instanceof Line)) {
			return false;
		}
		Line thisLine = (Line) thisObject;

		if (!thisLine.isSubSection(this)) {
			return false;
		}

		if (isSubSection(thisLine)) {
			return false;
		}

		// every point on either line is on the other line so they
		// must be equivlent lines
		return true;
	}
	
	/**{@inheritDoc}*/
	@Override
	public int hashCode() {
		return this.points.hashCode();
	}

}
