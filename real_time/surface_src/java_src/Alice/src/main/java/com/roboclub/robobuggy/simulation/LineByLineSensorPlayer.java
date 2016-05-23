package com.roboclub.robobuggy.simulation;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Date;

/**
 * 
 * @author Trevor Decker
 * a tool for reading log files line by line
 *
 */
public class LineByLineSensorPlayer {
	
	/**
	 * Constructor for sensor player
	 * @param filePath the file to play
	 * @param playBackSpeed the playback sped
	 */
	public LineByLineSensorPlayer(String filePath,double playBackSpeed) {
		//open up the log file 
		
		 new RobobuggyLogicNotification("initializing the SensorPlayer", RobobuggyMessageLevel.NOTE);
		 Thread thread = new Thread(){
			    public void run(){
			    	Gson translator = new GsonBuilder().create();//TODO don't create a new one for every call of this function
			    	// open input stream test.txt for reading purpose.
			    	try {
			    		InputStream is = new FileInputStream(filePath);
			    		// create new input stream reader
			    		InputStreamReader isr = new InputStreamReader(is);
			    		//the file was created so lets try and read it 
			    		BufferedReader br = new BufferedReader(isr);
			    		String nextLine;
			    			nextLine= br.readLine();  //reads {
			    			nextLine = br.readLine();  // name
			    			nextLine = br.readLine();  //schema version
			    			nextLine = br.readLine();  //date recorded
			    			nextLine = br.readLine();  //software version 
			    			nextLine = br.readLine();  //Sensor_data
			    			nextLine = br.readLine(); 
		    				nextLine = nextLine.substring(0, nextLine.length()-1);
			    			JsonObject sensorObject =  translator.fromJson(nextLine, JsonObject.class);
			    			long startTimeSensor = sensorObject.get("timestamp").getAsLong();
			    			
			    			long startTimeReal = new Date().getTime();
			    			nextLine = br.readLine(); 
			    			while(nextLine != null && !nextLine.startsWith("]")){
			    			try {
			    				//has not reached the end of the log yet
			    				//removes the comma at the end of the line
			    				nextLine = nextLine.substring(0, nextLine.length()-1);
			    				long now = new Date().getTime();
			    				 sensorObject =  translator.fromJson(nextLine, JsonObject.class);
			    				 long dt = now - startTimeReal;
			    				PlayBackUtil.parseSensorLog(sensorObject, translator,dt,startTimeSensor,
			    						RobobuggyConfigFile.getPlayBackSpeed());
			    				//TODO deal with delay stuff
							
			    				nextLine = br.readLine(); 
			    			} catch (IOException e) {
			    				// TODO Auto-generated catch block
			    				e.printStackTrace();
			    			} catch (InterruptedException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}//while

			
				} catch (FileNotFoundException e) {
		            new RobobuggyLogicNotification("File doesn't exist!", RobobuggyMessageLevel.EXCEPTION);
					e.printStackTrace();
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				    }//run

		 };//thread

				  thread.start();
	            
	       
	}
	

}
