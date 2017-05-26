package c_two_source_integrator.startercode;

public class MainFile {
	
	public static void main(String args[]) {
	
		new IntegralSink();
		new TwoSourceIntegrator();
		new NumberSource();
		// N. B. The order of these calls is important! why? 
		// 		What if they were in a different order? (bonus points if you
		//			find the relevant file in BuggyRos)
	}

}
