package com.roboclub.robobuggy.logging;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import javax.imageio.ImageIO;

import com.roboclub.robobuggy.ui.Gui;

/**
 * Logs data from the sensors
 * 
 * @author Joe Doyle
 */
public final class SensorLogger {
	private final String[] _imuKeys = {
		"IMU_Acc_X",     "IMU_Acc_Y",     "IMU_Acc_Z",
		"IMU_Gyro_X",    "IMU_Gyro_Y",    "IMU_Gyro_Z",
		"IMU_Compass_X", "IMU_Compass_Y", "IMU_Compass_Z"
	};
	private final String[] _encoderKeys = {
		"Encoder_Ticks","Encoder_Ticks_Total","Encoder_Time"
	};
	private final String[] _servoKeys = {
		"Servo_Angle_Command", "Servo_Angle_Actual"
	};
	private final String[] _gpsKeys = {
		"GPS_Longitude", "GPS_Latitude"
	};
	private final String[] _imgKeys = {
		"Image_Name"
	};
	
	private final String[][] _keySets = {
		_imuKeys,_encoderKeys,_servoKeys,_gpsKeys,_imgKeys	
	};
	
	private static final class _TaggedImage {
		public final String tag;
		public final BufferedImage img;
		public _TaggedImage(String tag,BufferedImage img) {
			this.tag = tag;
			this.img = img;
		}
	}

	private final PrintStream _csv;
	private final Queue<String[]> _csvQueue;
	private final Queue<_TaggedImage> _imgQueue;
	
	private final String[] _keys;

	private static final Queue<String[]> startCsvThread(PrintStream stream) {
		final LinkedBlockingQueue<String[]> ret = new LinkedBlockingQueue<>();
		new Thread() {
			public void run() {
				while(true) {
					try {
						String[] line = ret.take();
						if(line == null) {
							break;
						}
						boolean first = true;
						for(String s: line) {
							if(!first) {
								stream.print(",");
							} else {
								first = false;
							}
							stream.print(s);
						}
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		}.start();  //TODO please make eaiser to read
		return ret;
	};
	
	private static final Queue<_TaggedImage> startImgThread(File outDir) {
		final LinkedBlockingQueue<_TaggedImage> ret = new LinkedBlockingQueue<>();
		new Thread() {
			public void run() {
				while(true) {
					try {
						_TaggedImage img = ret.take();
						if(img == null) {
							break;
						}
						ImageIO.write(img.img, "jpg", new File(outDir,img.tag + ".jpg"));
					} catch (InterruptedException e) {
						e.printStackTrace();
 					} catch (IOException e) {
						e.printStackTrace();
					}
				}
			}
		}.start();
		return ret;
	};

	SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
	public SensorLogger(File outputDir,Date startTime) throws Exception {
		//check if dir exits
		if(outputDir == null){
			throw new Exception("Output Directory was null!");
		} else if (!outputDir.exists()) {
			outputDir.mkdirs();
		}
		String outputFileName = startTime.toString();
		outputFileName = outputFileName.replaceAll(" ","");
		outputFileName = outputFileName.replaceAll(":", "_");
		File csvFile = new File(outputDir,outputFileName + "sensors.csv");
		System.out.println("FileCreated: " + outputFileName);
		//TODO fix Gui.UpdateLogName( outputFileName );
		try {
			_csv = new PrintStream(csvFile);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException("Cannot create sensor log file (" + csvFile + ")!");
		}
		_csvQueue = startCsvThread(_csv);

		File imgdir = new File(outputDir,df.format(startTime) + "-images");
		imgdir.mkdirs();
		_imgQueue = startImgThread(imgdir);

		ArrayList<String> keys = new ArrayList<>();
		keys.add("Timestamp");
		for(String[] ks: _keySets) {
			for(String k: ks) {
				keys.add(k);
			}
		}
		_keys = new String[keys.size()];
		keys.toArray(_keys);
		_csvQueue.offer(_keys);
	}
	
	private void _log(long timestamp,String[] currkeys,String[] values) {
		String[] line = new String[_keys.length];
		for(int i=0;i<_keys.length;++i) {
			String k = _keys[i];
			if(k == "Timestamp") {
				line[i] = "" + timestamp;
				continue;
			}
			for(int j=0;j<currkeys.length;++j) {
				if(currkeys[j] == k) {
					line[i] = values[j];
				}
			}
		}
		_csvQueue.offer(line);
	}

	public void logImu(long time,float[] acc,float[] gyro,float[] compass) {
		String[] vals = new String[9];
		for(int i=0;i<3;++i) {
			vals[i+0] = "" + acc[i];
			vals[i+3] = "" + gyro[i];
			vals[i+6] = "" + compass[i];
		}
		_log(time,_imuKeys,vals);
	}
	public void logServo(long time,float command,float actual) {
		_log(time,_servoKeys,new String[]{ "" + command,"" + actual });
	}
	public void logEncoder(long time,long encTickLast,long encReset,long encoderTime) {
		_log(time,_encoderKeys,new String[]{ "" + encTickLast,"" + encReset,"" + encoderTime });
	}
	public void logGps(long time_in_millis,double d,double e) {
		_log(time_in_millis,_gpsKeys,new String[]{ "" + d,"" + e });
	}
	public void logImage(long time,String name,BufferedImage img) {
		_log(time,_imgKeys,new String[]{ "" + name });
		_imgQueue.offer(new _TaggedImage(name, img));
	}
}
