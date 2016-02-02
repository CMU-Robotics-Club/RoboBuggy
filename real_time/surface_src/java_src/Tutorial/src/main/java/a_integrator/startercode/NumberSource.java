package a_integrator.startercode;

import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;

public class NumberSource implements Node {

	Publisher p;

	@Override
	public void setName(String newName) {
		System.out.println("tried to set name");
	}

	@Override
	public String getName() {
		return null;
	}
	public NumberSource() {
		p = new Publisher("values"); 
		
		// Create a new thread that sends messages;
		//  (this function has to return before any other code gets run. 
		//    We want to publish messages later.)
		(new Thread(new Runnable() {
			int counter = 0;
			// The code you want to run after 'main' returns, put here.
			@Override
			public void run() {
				while(true) {
					IntegerMessage im = new IntegerMessage(this.counter);
					
					p.publish(im);
					
					counter++;
					
					try {
						Thread.sleep(1000 / 60);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
					
				}
			}
		})).run();
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
	
	
	
	
}

