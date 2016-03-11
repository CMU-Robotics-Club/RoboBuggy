package com.roboclub.robobuggy.nodes.localizers;

import java.util.ArrayList;

import Jama.Matrix;

/**
 * Observation model - TODO
 */
public abstract class ObservationModel {

	abstract Matrix getObservationSpaceState(Matrix state);

}
