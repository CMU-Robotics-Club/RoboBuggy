package com.roboclub.robobuggy.simulation;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

import java.util.Date;

import com.roboclub.robobuggy.fauxNodes.FauxNode;
import com.roboclub.robobuggy.fauxNodes.FauxEncoderNode;
import com.roboclub.robobuggy.fauxNodes.FauxGPSNode;
import com.roboclub.robobuggy.fauxNodes.FauxIMUNode;
import com.roboclub.robobuggy.fauxNodes.FauxSteeringNode;
import com.roboclub.robobuggy.ros.SensorChannel;

public class FauxRunner implements Runnable {

	private FauxIMUNode imu = null;
	private FauxGPSNode gps = null;
	private FauxEncoderNode enc = null;
	private FauxSteeringNode drive_ctrl = null;
	private String path;
	
	//Can probably improve the constructor to take all collections, but meh.
	public FauxRunner(ArrayList<FauxNode> sensors, String path) {
		
		//really, really ghetto. Sorry.
		this.path = path;
		for (FauxNode sensor : sensors) {
			switch (sensor.typeString) {
			case "IMU":
				imu = (FauxIMUNode) sensor;
				break;
			case "GPS":
				gps = (FauxGPSNode) sensor;
				break;
			case "ENC":
				enc = (FauxEncoderNode) sensor;
				break;
			case "DRIVE_CTRL":
				drive_ctrl = (FauxSteeringNode) sensor;
				break;
			default:
				continue;
			}
		}
	}
	
	private static long getOffset(String line) {
		String[] parsed = line.split(",");
		
		assert parsed.length >= 2;
		
		Date date = FauxNode.makeDate(parsed[1]);
		return FauxNode.calculateOffset(new Date(), date);		
	}

	@Override
	public void run() {
		// TODO Auto-generated method stub
		
		System.out.println("Starting to read from file!");

		try(BufferedReader br = new BufferedReader(new FileReader(path))) {
			String line = br.readLine();
			
			long offset = getOffset(line);
			FauxNode.setOffset(offset);
			
			while (true) {
				if (line == null) { 
					System.out.println("Reached end of file!");
					return;
				}
				
				int endIndex = line.indexOf(',');
				if (endIndex == -1) {
					//Skipping line!
					line = br.readLine();
					continue;
				} else {
					// TODO: Fix crashing when not all of the sensors are defined.
					String sensor = line.substring(0, endIndex);
					switch (sensor.toLowerCase()) {
					case "sensors/imu":
						if (imu != null) {
							imu.qAdd(line);
						}
						break;
					case "sensors/steering":
						if (drive_ctrl != null) {
							drive_ctrl.qAdd(line);
						}
						break;
					case "sensors/encoder":
						if (enc != null) {
							enc.qAdd(line);
						}
						break;
					case "sensors/gps":
						if (gps != null) {
							gps.qAdd(line);
						}
						break;
//					case "sensors/logging_button":
//						lsp.qAdd(line);
//						break;
//					case "sensors/brake":
//						bp.qAdd(line);
//						break;
					default:
//						System.out.println("Unknown sensor type: " + sensor.toLowerCase());
						break;
					}
					line = br.readLine();
				}
			}
		} catch (Exception e) {
			System.out.println("Looks like there was a problem somewhere in reading the file?.");
			e.printStackTrace();
		}		
	}
}