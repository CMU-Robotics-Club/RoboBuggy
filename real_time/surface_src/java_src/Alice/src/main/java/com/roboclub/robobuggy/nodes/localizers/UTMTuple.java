package com.roboclub.robobuggy.nodes.localizers;



/**
 * Private class used for representing a geospatial point in UTM coordinates
 *
 * @author Ian Hartwig
 */
public class UTMTuple {
	public int Zone;
    public char Letter;
	public double Easting;
	public double Northing;
	
	/**
	 * Construct a new {@link UTMTuple}
	 * @param Zone Zone
	 * @param Letter Letter
	 * @param Easting Easting
	 * @param Northing Northing
	 */
	public UTMTuple(int Zone, char Letter, double Easting, double Northing){
		this.Zone = Zone;
		this.Letter = Letter;
		this.Easting = Easting;
		this.Northing = Northing;
	}
}