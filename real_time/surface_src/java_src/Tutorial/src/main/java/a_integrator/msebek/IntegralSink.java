package a_integrator.msebek;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Subscriber;

public class IntegralSink implements Node {

	Subscriber s = new Subscriber("integratedValues", new MessageListener() {
		@Override
		public void actionPerformed(String topicName, Message m) {
			// Cast m to the correct message type. If this cast fails, an exception is thrown.
			IntegerMessage im = (IntegerMessage) m;
			System.out.println("Hello, BuggyRos! (" + im.val + ")");
		}
	});
	
	@Override
	public boolean shutdown() {
		// no resources to clean up; simply return.
		return true;
	}

	@Override
	public boolean startNode() {
		// TODO Auto-generated method stub
		return false;
	}
	
	
	
}

