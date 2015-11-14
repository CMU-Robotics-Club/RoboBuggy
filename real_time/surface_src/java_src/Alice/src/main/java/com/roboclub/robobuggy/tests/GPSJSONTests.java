package com.roboclub.robobuggy.tests;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.nodes.GpsNode;

public class GPSJSONTests {

	public static void main(String[] args) {
		JSONObject o = new JSONObject();
		o = GpsNode.translatePeelMessageToJObject("sensors/gps,2015-10-17 06:58:28.999,2015-10-17 10:58:04.959,40.43345327777995,N,79.94215965270996,W,1,5,6.59,347.1");
		System.out.println(o.toJSONString());
	}
	
}
