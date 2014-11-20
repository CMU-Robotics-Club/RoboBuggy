package com.roboclub.robobuggy.ros;

import java.text.Format;
import java.text.SimpleDateFormat;

/**
 * @author Matt Sebek
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

public interface Message {
	static final Format formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
	
	public String toLogString();

	public void fromLogString(String str);
}
