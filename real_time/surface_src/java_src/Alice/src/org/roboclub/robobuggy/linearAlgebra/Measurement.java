package org.roboclub.robobuggy.linearAlgebra;

/**
 * TODO document
 * @author Trevor Decker
 *
 */
public abstract class Measurement<NTYPE extends Number> implements Number{
	NTYPE value;

	
	@Override
	/**
	 * TODO document
	 * TODO include
	 */
	public Number sqrt() {
		value.sqrt();
		return null;
	}

	@Override
	/**
	 * TODO document
	 */
	public Measurement signum() throws CloneNotSupportedException {
		Measurement<NTYPE> result = (Measurement<NTYPE>) this.clone();
		result.value = (NTYPE) this.value.signum();
		return result;
	}
}
