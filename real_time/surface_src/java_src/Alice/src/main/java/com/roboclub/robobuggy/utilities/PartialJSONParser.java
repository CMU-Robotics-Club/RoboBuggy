package com.roboclub.robobuggy.utilities;

import java.util.HashMap;

/**
 * Class used as a partial JSON parser
 */
public class PartialJSONParser {
    private HashMap<String, String> delims;

    /**
     * Construct a new {@link PartialJSONParser}
     *
     * @param filePath {@link String} filepath to convert
     */
    public PartialJSONParser(String filePath) {
        delims = new HashMap<>();
        delims.put("{", "}");
        delims.put("[", "]");
        delims.put("]", "[");
        delims.put("}", "{");
    }
}
