package b_rolling_average.startercode;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

public class RollingAverager implements Node {

	Publisher p;
	Subscriber s;

	// Note that if 'NumRollingElems' is 5, then the average of the last 5 
	//		elements should be output.
	public RollingAverager(int NumRollingElems) {
		// Create a subscriber that listens on the channel of NumberSource.java
		//  Every time a message comes in, get this integer, and add it to the sum.
		// Publish this sum on to the channel RollingAverageSink is listening to.
	
		// If you notice any limitations/ambiguous points, make a design
		//	decision and document it. Better decisions lead to a better
		//  autonomous buggy.
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

