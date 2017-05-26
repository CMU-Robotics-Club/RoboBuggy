package c_two_source_integrator.startercode;

import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;

public class NumberSource implements Node {
	
	Publisher p;

	public NumberSource() {
		p = new Publisher("values"); 
	
		// TODO you: finish this file; too!
	}
	
	@Override
	public boolean shutdown() {
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

