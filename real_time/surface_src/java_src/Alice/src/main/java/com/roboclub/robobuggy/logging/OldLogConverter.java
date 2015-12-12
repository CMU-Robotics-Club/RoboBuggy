package com.roboclub.robobuggy.logging;
import java.util.*;
import java.io.*;


public class OldLogConverter {
	
/** Steering Angle Conversion Rate */
static private final int ARD_TO_DEG = 100;
/** Steering Angle offset?? */
static private final int OFFSET = -200;
	
static private ArrayList<String> sensor_type = new ArrayList<String>();
static private ArrayList<Integer> sensor_quant = new ArrayList<Integer>();

	public static void main(String[] args) throws IOException {
		convertLog("/Users/davidneiman/Desktop/sensors.txt", "/Users/davidneiman/Desktop/sensorsJSON.txt");
	}
	
	public static void convertLog(String inputpath, String outputpath) throws FileNotFoundException{		
		Scanner scanner = new Scanner(new File(inputpath));
		PrintStream writer = new PrintStream(new File(outputpath));

		
		//Read in the first set of data so we get the date we're recording on
		String s = scanner.nextLine();
		StringTokenizer st = new StringTokenizer(s, ",");
		String type = st.nextToken().substring(8); //Could be IMU, GPS, encoder, steering
		StringTokenizer st2 = new StringTokenizer(st.nextToken(), " ");
		//The next token is the date, but in yyyy-mm-dd format; I want mm/dd/yyyy format
		String date = st2.nextToken();
		StringTokenizer stdate = new StringTokenizer(date, "-");
		String[] dateymd = {stdate.nextToken(), stdate.nextToken(), stdate.nextToken()};
		String date_recorded = dateymd[1] + "/" + dateymd[2] + "/" + dateymd[0]; //This should now be in mm/dd/yyyy format
		String timestamp = date + " " + st2.nextToken();
		
		//Add heading
		String logname = "Robobuggy Data Logs";
		String schema_version = "1.0";
		String software_version = "1.0.0";
		writer.println("{");
		writer.println("    \"name\": \"" + logname + "\",");
		writer.println("    \"schema_version\": " + schema_version + ",");
		writer.println("    \"date_recorded\": \"" + date_recorded + "\",");
		writer.println("    \"software_version\": \"" + software_version + "\",");
		writer.println("    \"sensor_data\": [");
		
		//Write the first set of data we read in before
		writeSensorData(scanner, writer, st, type, timestamp);
		
		//Main loop to log sensor data
		while(scanner.hasNextLine()){
			s = scanner.nextLine();
			st = new StringTokenizer(s, ",");
			type = st.nextToken().substring(8); //Could be IMU, GPS, encoder, steering
			st2 = new StringTokenizer(st.nextToken(), " ");
			timestamp = st2.nextToken() + " " + st2.nextToken(); //Date time
			writeSensorData(scanner, writer, st, type, timestamp);
		}
		//End sensor data part
		writer.println("    ],");
		
		//Print the breakdown
		writer.print("    \"data_breakdown\" : {");
		for(int x = 0; x < sensor_type.size(); x++){
			writer.print("\"" + sensor_type.get(x) + "\":" + sensor_quant.get(x));
			//Omit last comma
			if(x < sensor_type.size()-1){
				writer.print(",");
			}
		}
		writer.println("}");
		
		//Clean up and end
		writer.print("}");
		System.out.println("Done converting");
	}
	
	//Update the types of sensors we have readings for as well as the number of readings
	public static void updateDataBreakdown(ArrayList<String> sensor_type, ArrayList<Integer> sensor_quant, String type){
		//If you don't have the sensor, add it to the list and note that you now have 1 reading for it
		if(sensor_type.indexOf(type) == -1){
			sensor_type.add(type);
			sensor_quant.add(1);
		}
		//Otherwise, increment the number of readings you have
		else{
			sensor_quant.set(sensor_type.indexOf(type), sensor_quant.get(sensor_type.indexOf(type)).intValue()+1);
		}
	}
	
	//Given the necessary data, write it to a file in the new format. Lots of brute-force and case statements here
	//Note: Anything that would be a tab is a quadruple space
	//Another note: Capitalization of sensor names is off
	public static void writeSensorData(Scanner scanner, PrintStream writer, StringTokenizer st, String name, String timestamp){
		String fname = formattedSensorName(name);
			writer.print("        {\"name\":\"" + fname + "\",");
			printSensorData(fname, st, writer);
			//Appending the timestamp to the end of everything is universal 
			writer.print("\"timestamp\":\"" + timestamp + "\"}");
			if(scanner.hasNextLine()){
				writer.println(",");
			}
			else{
				writer.println();
			}
		updateDataBreakdown(sensor_type, sensor_quant, fname);
	}
	
	public static String formattedSensorName(String name){
		//Returns the sensor name in the format we're using in the new logs
		//Format: First letter capitalized except for acronyms like IMU and the name logging button, which is weird
		if("GPS".equalsIgnoreCase(name)){return("GPS");}
		if("IMU".equalsIgnoreCase(name)){return("IMU");}
		if("logging_button".equals(name)){return("logging button");}
		else{return(name.substring(0, 1).toUpperCase() + name.substring(1));}
	}
	
	public static void printSensorData(String sensorname, StringTokenizer st, PrintStream writer){
		//Checks for all possible sensors and outputs the appropriate parameters
		switch(sensorname){
		case "Steering":
			String angle = st.nextToken();
			Double potValue = Double.parseDouble(angle);
			potValue = -(potValue + OFFSET)/ARD_TO_DEG;
			angle = potValue.toString();
			writer.print("\"params\":{\"angle\": " + angle + "},");
			break;
		case "Encoder":
			String dataword = st.nextToken();
			String d = st.nextToken();
			String v = st.nextToken();
			String a = st.nextToken();
			if("NaN".equals(dataword) || dataword.contains("Infinity")){
				dataword = "null";
			}
			if("NaN".equals(d) || d.contains("Infinity")){
				d = "null";
			}
			if("NaN".equals(v) || v.contains("Infinity")){
				v = "null";
			}
			if("NaN".equals(a) || a.contains("Infinity")){
				a = "null";
			}
			writer.print("\"params\":{\"dataword\":" + dataword + ",\"distance\":" + d + ",\"velocity\":" + v + ",\"acceleration\":" + a + "},");
			break;
		case "logging button":
			writer.print("\"params\":{\"logging_status\":\"" + st.nextToken().toLowerCase() + "\"},");
			break;
		case "GPS":
			st.nextToken(); //GPS data stores a second time; I'm not really sure what it is, but we don't need it
			//Latitude/longitude
			writer.print("\"params\":{\"latitude\":" + st.nextToken() + ",\"lat_direction\":\"" + st.nextToken() + "\",\"longitude\":" + st.nextToken() + ",\"long_direction\":\"" + st.nextToken() + "\",");
			//Insert raw longitude/latitude stuff. There's no data in the old files, so use -1.
			writer.print("\"raw_gps_lon\":-1,\"raw_gps_lat\":-1,");
			//Other parameters
			writer.print("\"gps_quality\":\"" + st.nextToken() + "\",\"num_satellites\":\"" + st.nextToken() + "\",\"HDOP\":" + st.nextToken() + ",\"antenna_altitude\":" + st.nextToken() + "},");
			break;
		case "IMU":
			//Note: I'm not sure what order these read in in. I'm taking an educated guess that the -120 reading is yaw and that therefore the old data is YPR
			String yaw = st.nextToken();
			String pitch = st.nextToken();
			String roll = st.nextToken();
			writer.print("\"params\":{\"roll\":" + roll + ",\"pitch\":" + pitch + ",\"yaw\":" + yaw + "},");
			break;
		default:
			//Note: This doesn't print any parameters, but you'll still have the sensor name and timestep in the log
			System.out.println("Unknown sensor: " + sensorname);
			break;
		}
	}
}
