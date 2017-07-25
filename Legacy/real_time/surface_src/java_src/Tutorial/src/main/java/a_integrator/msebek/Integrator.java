package a_integrator.msebek;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

public class Integrator implements Node {

	Publisher p;
	Subscriber s;
	int sum = 0;
	
	public Integrator() {
		this.p = new Publisher("integratedValues");

		this.s = new Subscriber("integrator", "values", new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				IntegerMessage im = (IntegerMessage) m;
				sum += im.val;
				p.publish(new IntegerMessage(sum));
			}
		});
	}

	
	@Override
	public boolean shutdown() {
		// No resources-needing-closing required.
		return true;
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

