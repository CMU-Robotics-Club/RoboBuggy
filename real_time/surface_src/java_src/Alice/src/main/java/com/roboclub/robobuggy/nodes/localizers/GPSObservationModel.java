package com.roboclub.robobuggy.nodes.localizers;

import java.util.ArrayList;

public class GPSObservationModel {

	 ArrayList getObservationSpaceState(ArrayList state){
		 ArrayList output = new ArrayList();
		 output.add(state.get(0));
		 output.add(state.get(1));
		return output;
	 }
	
}
