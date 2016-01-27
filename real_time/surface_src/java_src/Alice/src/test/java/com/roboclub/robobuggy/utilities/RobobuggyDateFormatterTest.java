package com.roboclub.robobuggy.utilities;

import static org.junit.Assert.*;

import java.util.Date;

import org.junit.Test;

/**
 * Class used to test {@link RobobuggyDateFormatter}
 * 
 * @author Zachary Dawson
 */
public class RobobuggyDateFormatterTest {

	/**
	 * Test with null input
	 */
	@Test
	public void testNull() {
		Date result = RobobuggyDateFormatter.formatRobobuggyDate("2016-12-22 12:12:12.123");
		assertTrue(result != null);
	}

}
