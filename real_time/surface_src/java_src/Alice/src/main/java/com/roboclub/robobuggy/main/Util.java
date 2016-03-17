package com.roboclub.robobuggy.main;

import gnu.io.CommPortIdentifier;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;

import Jama.Matrix;

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
        JsonObject jsonRefrence = translator.fromJson(fileReader, JsonObject.class);
		return jsonRefrence;

    }
	
	
	
}
