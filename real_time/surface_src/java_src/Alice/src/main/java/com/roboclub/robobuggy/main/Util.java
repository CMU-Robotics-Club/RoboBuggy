package com.roboclub.robobuggy.main;

import Jama.Matrix;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;

import gnu.io.CommPortIdentifier;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

/**
 * Util - Matrix utilities and other utilities
 */
public final class Util {

	/**
	 * creates the identity matrix
	 * @param size size of the matrix
	 * @return the identity matrix
	 */
	public static Matrix createIdentityMatrix(int size){
		Matrix result = new Matrix(size,size,0.0);
		for(int i = 0;i<size;i++){
			result.set(i,i,1.0);
		}
		return result;
	}

	/**
	 * Evaluates to a List of the com ports(serial) which are available on the computer 
	 * @return List<String> of available com ports
	 */
    public static List<String> getAvailablePorts() {

        List<String> list = new ArrayList<String>();

        Enumeration<?> portList = CommPortIdentifier.getPortIdentifiers();

        while (portList.hasMoreElements()) {
            CommPortIdentifier portId = (CommPortIdentifier) portList.nextElement();
            if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
                list.add(portId.getName());
            }
        }

        return list;
    }
    
    /**
     * This is a helper function which evaluates to a Json object encoding the same 
     * information as the file which was passed to this function
     * @param path a string of the path to a file containing a Json object
     * @return JsonObject of the contents of the file 
     * @throws UnsupportedEncodingException if the file is not JSON
     * @throws FileNotFoundException if the file is not found
     */
	public static JsonObject readJSONFile(String path) throws UnsupportedEncodingException, FileNotFoundException{
	       Gson translator = new GsonBuilder().create();
		   InputStreamReader fileReader = new InputStreamReader(new FileInputStream(new File(path)), "UTF-8");
		return translator.fromJson(fileReader, JsonObject.class);

    }
	
	
	/**
	 * Normalizes an an input angle in degrees to be between -180 and 180
	 * @param degrees input angle in degrees
	 * @return normalized angle in degrees 
	 */
	public static double normalizeAngleDeg(double degrees){
		//java mod keeps the sign of the dividend so lets make everything positive
		while(degrees < 0.0 ){
			degrees = degrees+360;
		}
		degrees = degrees % 360.0;
		if(degrees > 180.0 ){
			degrees = degrees - 360.0;
		}
		return degrees;
		
	}
	
	/**
	 * Normalizes an an input angle in degrees to be between -pi and pi
	 * @param radians input angle in radians
	 * @return normalized angle in radians 
	 */
	public static double normalizeAngleRad(double radians){
		//java mod keeps the sign of the dividend so lets make everything positive
		while(radians < 0.0 ){
			radians = radians+2*Math.PI;
		}

		radians = radians % (2*Math.PI);

		if(radians > Math.PI ){
			radians = radians - 2.0*Math.PI;
		}
		return radians;
		
	}
	
	
	
}
