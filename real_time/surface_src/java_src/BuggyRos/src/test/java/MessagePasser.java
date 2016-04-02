

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

public class MessagePasser implements Node {

	Publisher p;
	Subscriber s;

	public MessagePasser(String from, String to) {
		p = new Publisher(to);
		s = new Subscriber(from, new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				p.publish(m);
			}
		});
	}
	
	@Override
	public boolean shutdown() {
		return true;
	}

	@Override
	public boolean startNode() {
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