package b_rolling_average.startercode;

public class MainFile {
	
	public static void main(String args[]) {
	
		// Note that we don't save the returned handle. This is because, once
		//  a node has been started, BuggyRos takes a handle to the publishers
		//  and the subscribers. Since these are in use, the rest of the class
		//  won't be garbage collected, which is what we want. 
		//
		// This is pretty cool, since even if you ignore/don't understand any
		//   of that, 'the right thing' will still happen. Can't do that in 
		//   C/C++ now, could you?
		//
		// (note that main will return & exit before any messages are even sent!)
		new RollingAverageSink();
		new RollingAverager(10);
		new NumberSource();
	}

}
