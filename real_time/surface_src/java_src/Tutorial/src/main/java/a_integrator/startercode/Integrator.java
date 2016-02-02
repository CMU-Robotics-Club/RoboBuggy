package a_integrator.startercode;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

public class Integrator implements Node {

	Publisher p;
	Subscriber s;
	
	public Integrator() {
		// Create a subscriber that listens on the channel of NumberSource.java
		//  Every time a message comes in, get this integer, and add it to the sum.
		// Publish this sum on to the channel IntegralSink is listening to.
	}

	
	@Override
	public boolean shutdown() {
		// No resources-needing-closing required; shutdown.
		//   (but if you opened a file or something, close it cleanly)
		return true;
	}


	@Override
	public boolean startNode() {
		// TODO Auto-generated method stub
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

	
	
}

