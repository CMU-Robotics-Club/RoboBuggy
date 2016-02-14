package com.roboclub.robobuggy.utilities;

import static org.junit.Assert.*;

import java.util.Date;

import com.roboclub.robobuggy.messages.BaseMessage;
import org.junit.Test;

/**
 * Class used to test {@link com.roboclub.robobuggy.messages.BaseMessage}
 * 
 * @author Zachary Dawson
 */
public class RobobuggyDateFormatterTest {

	/**
	 * Test with null input
	 */
	@Test
	public void testNull() {
		Date result = BaseMessage.tryToParseDate("2016-12-22 12:12:12.123");
		assertTrue(result != null);
	}

}
