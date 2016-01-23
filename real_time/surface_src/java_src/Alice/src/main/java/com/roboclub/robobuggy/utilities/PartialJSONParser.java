package com.roboclub.robobuggy.utilities;

import java.util.HashMap;

public class PartialJSONParser {
	private HashMap<String, String> delims;

	public PartialJSONParser(String filePath) {
		delims = new HashMap<>();
		delims.put("{", "}");
		delims.put("[", "]");
		delims.put("]", "[");
		delims.put("}", "{");
	}
}
