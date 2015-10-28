package com.roboclub.robobuggy.nodes;

import com.orsoncharts.util.json.JSONObject;

public class GPSNodeTester {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		JSONObject o = GpsNode.translatePeelMessageToJObject("sensors/gps,2015-10-04 06:09:33.884,2015-10-04 10:09:01.884,40.43345459721982,N,79.94210414886474,W,1,4,15.76,306.1");
		System.out.println(o.toJSONString());;
	}

}
