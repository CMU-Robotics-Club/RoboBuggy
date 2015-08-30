package org.roboclub.robobuggy.simulation;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

import java.util.Date;

import org.roboclub.robobuggy.fauxNodes.FauxNode;
import org.roboclub.robobuggy.fauxNodes.FauxEncoderNode;
import org.roboclub.robobuggy.fauxNodes.FauxGPSNode;
import org.roboclub.robobuggy.fauxNodes.FauxIMUNode;
import org.roboclub.robobuggy.fauxNodes.FauxSteeringNode;
import org.roboclub.robobuggy.ros.SensorChannel;

public class FauxRunner implements Runnable {

	private FauxIMUNode imu = null;
	private FauxGPSNode gps = null;
	private FauxEncoderNode enc = null;
	private FauxSteeringNode drive_ctrl = null;
	private String path;
	
	public FauxRunner(ArrayList<FauxNode> sensors, String path) {
		
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

		//TODO: Make this take in a file argument instead of being hardcoded
//		try(BufferedReader br = new BufferedReader(new FileReader("logs\\2015-04-12-06-22-37\\sensors.txt"))) {
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
//						throw new Exception("Malformed data!");
				} else {
					String sensor = line.substring(0, endIndex);
					switch (sensor.toLowerCase()) {
					case "sensors/imu":
						imu.qAdd(line);
						break;
					case "sensors/steering":
						drive_ctrl.qAdd(line);
						break;
					case "sensors/encoder":
						enc.qAdd(line);
						break;
					case "sensors/gps":
						gps.qAdd(line);
						break;
//					case "sensors/logging_button":
//						lsp.qAdd(line);
//						break;
//					case "sensors/brake":
//						bp.qAdd(line);
//						break;
					default:
						System.out.println("Unknown sensor type: " + sensor.toLowerCase());
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