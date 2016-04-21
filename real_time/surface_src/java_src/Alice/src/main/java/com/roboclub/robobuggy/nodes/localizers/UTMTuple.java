package com.roboclub.robobuggy.nodes.localizers;



/**
 * Private class used for representing a geospatial point in UTM coordinates
 *
 * @author Ian Hartwig
 */
public class UTMTuple {
	private int zone;
    private char letter;
	private double easting;
	private double northing;
	
	/**
	 * Construct a new {@link UTMTuple}
	 * @param zone Zone
	 * @param letter Letter
	 * @param easting Easting
	 * @param northing Northing
	 */
	public UTMTuple(int zone, char letter, double easting, double northing){
		this.setZone(zone);
		this.setLetter(letter);
		this.setEasting(easting);
		this.setNorthing(northing);
	}

	/**
	 * @return the zone
	 */
	public int getZone() {
		return zone;
	}

	/**
	 * @param zone the zone to set
	 */
	public void setZone(int zone) {
		this.zone = zone;
	}

	/**
	 * @return the letter
	 */
	public char getLetter() {
		return letter;
	}

	/**
	 * @param letter the letter to set
	 */
	public void setLetter(char letter) {
		this.letter = letter;
	}

	/**
	 * @return the easting
	 */
	public double getEasting() {
		return easting;
	}

	/**
	 * @param easting the easting to set
	 */
	public void setEasting(double easting) {
		this.easting = easting;
	}

	/**
	 * @return the northing
	 */
	public double getNorthing() {
		return northing;
	}

	/**
	 * @param northing the northing to set
	 */
	public void setNorthing(double northing) {
		this.northing = northing;
	}
}