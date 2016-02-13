package com.roboclub.robobuggy.nodes.localizers;

import java.util.ArrayList;

import Jama.Matrix;

public abstract class ObservationModel {

	abstract Matrix getObservationSpaceState(Matrix State);

}
