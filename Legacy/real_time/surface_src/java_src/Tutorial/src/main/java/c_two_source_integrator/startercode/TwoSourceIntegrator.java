package c_two_source_integrator.startercode;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

public class TwoSourceIntegrator implements Node {

	Publisher p;
	Subscriber s1;
	Subscriber s2;

	public TwoSourceIntegrator() {
		// Create a subscriber that listens on the channel of NumberSource.java
		//  Every time a message comes in, get this integer, and add it to the sum.
		// Publish this sum on to the channel IntegralSink is listening to.
	
		// If you notice any limitations/ambiguous points, make a design
		//	decision and document it. Better decisions lead to a better
		//  autonomous buggy.
	}

	@Override
	public boolean shutdown() {
		// TODO Auto-generated method stub
		// Is this right? should this be changed?
		return false;
	}

	@Override
	public void setName(String newName) {
		System.out.println("tried to set name");
	}

	@Override
	public String getName() {
		return null;
	}

	@Override
	public boolean startNode() {
		// TODO Auto-generated method stub
		return false;
	}

	
	
	
}

