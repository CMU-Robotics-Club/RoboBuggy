import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class UnboundedMessageQueueTest {

	@Before
	public void setUp() throws Exception {
	}

	@After
	public void tearDown() throws Exception {
	}

	@Test
	public void onePubOneSub() {
		int numIterations = 100;
		UnboundedQueueNumberSink sink = new UnboundedQueueNumberSink("to", numIterations);
	
		// Publish as fast as we can!
		NumberSource source = new NumberSource("to", 0, numIterations);

		System.out.println("Blocking on done...");
		int count = sink.blockUntilDone();
		if(count != numIterations) {
			fail("Counts did not match");
		}
		
		source.shutdown();
		sink.shutdown();
	}

	@Test
	public void onePubOneSubWithMiddleNode() {
		int numIterations = 100;
		UnboundedQueueNumberSink sink = new UnboundedQueueNumberSink("to", numIterations);
		MessagePasser ms = new MessagePasser("middle", "from", "to");
	
		// Publish as fast as we can!
		NumberSource source = new NumberSource("from", 0, numIterations);

		System.out.println("Blocking on done...");
		int count = sink.blockUntilDone();
		if(count != numIterations) {
			fail("Counts did not match");
		}
	
		sink.shutdown();
		ms.shutdown();
		source.shutdown();
	}

	@Test
	public void onePubOneSubWithTwoMiddleNode() {
		int numIterations = 100;
		UnboundedQueueNumberSink sink = new UnboundedQueueNumberSink("to", numIterations);
		MessagePasser ms1 = new MessagePasser("firstPasser", "from", "middle");
		MessagePasser ms2 = new MessagePasser("secondPasser", "middle", "to");
	
		// Publish as fast as we can!
		NumberSource source = new NumberSource("from", 0, numIterations);

		System.out.println("Blocking on done...");
		int count = sink.blockUntilDone();
		if(count != numIterations) {
			fail("Counts did not match");
		}
		
		sink.shutdown();
		ms1.shutdown();
		ms2.shutdown();
		source.shutdown();
	}
}
