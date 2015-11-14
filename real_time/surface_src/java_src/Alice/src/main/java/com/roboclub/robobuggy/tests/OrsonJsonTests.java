package com.roboclub.robobuggy.tests;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import com.orsoncharts.util.json.JSONObject;
import com.orsoncharts.util.json.parser.JSONParser;
import com.orsoncharts.util.json.parser.ParseException;

public class OrsonJsonTests {

	
	public static void main(String[] args) {
		try {
			JSONParser parser = new JSONParser();
			FileReader reader = new FileReader(new File("C:\\Users\\Robot\\Documents\\GitHub\\RoboBuggy\\real_time\\surface_src\\java_src\\Alice\\src\\main\\java\\com\\roboclub\\robobuggy\\tests\\testLog.txt"));
			JSONObject obj = (JSONObject) parser.parse(reader);
			obj = null; //because we need something to break on
		} catch (IOException | ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
}
