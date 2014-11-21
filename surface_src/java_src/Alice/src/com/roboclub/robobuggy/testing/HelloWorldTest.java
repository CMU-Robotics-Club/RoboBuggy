import org.junit.Test;

public class HelloWorldTest extends junit.framework.TestCase {

	public void testNothing() {
	}

	public void testWillAlwaysFail() {
		fail("An error message");
	}

	public void testAnotherFail(){
		System.out.println("this should fail \n");
		fail("another error");
	}

}
