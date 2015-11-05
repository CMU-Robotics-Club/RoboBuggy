package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.nodes.RealNodeEnum;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorManager;
import com.roboclub.robobuggy.ui.Gui;

<<<<<<< HEAD
public class mainFile 
{
	static Robot buggy;
	static int num = 0;
	
	public static void main(String args[]) 
	{
		config.getInstance();//must be run at least once
		
		System.out.println(Paths.get("").toAbsolutePath().toString());
		System.err.println(Paths.get("").toAbsolutePath().toString());
		
		for (int i = 0; i < args.length; i++) 
		{
			if (args[i].equalsIgnoreCase("-g")) 
			{
				config.GUI_ON = false;
			} 
			else if (args[i].equalsIgnoreCase("+g")) 
			{
				config.GUI_ON = true;
			} 
			else if (args[i].equalsIgnoreCase("-r")) 
			{
				config.active = false;
			} 
			else if (args[i].equalsIgnoreCase("+r")) 
			{
				config.active = true;
			}
		}
		
		if(config.GUI_ON)
		{
			Gui.getInstance();
		}
		
		// Starts the robot
		if(config.DATA_PLAY_BACK_DEFAULT) //Changed this line
		{
			try 
			{
				bringup_sim();
			} 
			catch (Exception e) 
			{
				Gui.close();
				System.out.println("Unable to bringup simulated robot. Stacktrace omitted because it's really big.");
				e.printStackTrace();
				return;
			}
		} 
		else 
		{
			Robot.getInstance();
		}	
	}
	
	// Open a serial port
	private static SerialPort connect(String portName) throws Exception
    {
        CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
        if ( portIdentifier.isCurrentlyOwned() )
        {
            System.out.println("Error: Port is currently in use");
            return null;
        }
        else
        {
        	//TODO fix this so that it is not potato 
            CommPort commPort = portIdentifier.open("potato", 2000);
            
            if ( commPort instanceof SerialPort )
            {
                SerialPort serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(57600,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
                return serialPort;
=======
public class mainFile {
    static Robot buggy;
    static int num = 0;
    
    public static void main(String args[]) {
        config.getInstance();//must be run at least once
                
        for (int i = 0; i < args.length; i++) {
            if (args[i].equalsIgnoreCase("-g")) {
                config.GUI_ON = false;
            } else if (args[i].equalsIgnoreCase("+g")) {
                config.GUI_ON = true;
            } else if (args[i].equalsIgnoreCase("-r")) {
                config.active = false;
            } else if (args[i].equalsIgnoreCase("+r")) {
                config.active = true;
>>>>>>> master
            }
        }
        
        if(config.GUI_ON){
            Gui.getInstance();
        }
        
        // Starts the robot
        if(config.DATA_PLAY_BACK_DEFAULT){
            try {
                bringup_sim();
            } catch (Exception e) {
                Gui.close();
                System.out.println("Unable to bringup simulated robot. Stacktrace omitted because it's really big.");
                e.printStackTrace();
                return;
            }
        } else {
            Robot.getInstance();
        }   
    }
    
    //going to start by just connecting to the IMU
    public static void bringup_sim() throws Exception {
        if(config.logging){
            System.out.println("Starting Logging");
            RobotLogger.getInstance();
        }
<<<<<<< HEAD
		return null;
    }	
	public static void bringup_sim() throws Exception 
	{
		ArrayList<Node> sensorList = new ArrayList<Node>();

		// Turn on logger!
		if(config.logging)
		{
			System.out.println("Starting Logging");
			RobotLogger.getInstance();
		}

		Gui.EnableLogging();

		//setup objects for each of the driver nodes 
		ImuNode imu = new ImuNode(SensorChannel.IMU);
		GpsNode gps = new GpsNode(SensorChannel.GPS);
		RBSMNode enc = new RBSMNode(SensorChannel.ENCODER,SensorChannel.STEERING);
		// Set up the IMU
		SerialPort sp = null;
		String com = config.COM_PORT_IMU;
		try 
		{
			System.out.println("Initializing IMU Serial Connection");
			sp = connect(com);
			System.out.println("IMU connected to " + com);
		} 
		catch (Exception e) 
		{
			System.out.println("Unable to connect to necessary device on " + com);
			e.printStackTrace();
			throw new Exception("Device not found error");
		}
		imu.setSerialPort(sp);
		sensorList.add(imu);

		// Set up the GPS
		com = config.COM_PORT_GPS_INTEGRATED;
		try 
		{
			System.out.println("Initializing GPS Serial Connection");
			sp = connect(com);
			System.out.println("GPS connected to " + com);
		} 
		catch (Exception e) 
		{
			System.out.println("Unable to connect to necessary device on " + com);
			e.printStackTrace();
			throw new Exception("Device not found error");
		}
		gps.setSerialPort(sp);
		sensorList.add(gps);
	
		// Set up the Encoder
		com = config.COM_PORT_ENCODER;
		try 
		{
			System.out.println("Initializing ENCODER Serial Connection");
			sp = connect(com);
			System.out.println("ENCODER connected to " + com);
		} 
		catch (Exception e) 
		{
			System.out.println("Unable to connect to necessary device on " + com);
			e.printStackTrace();
			throw new Exception("Device not found error");
		}
		enc.setSerialPort(sp);
		sensorList.add(enc);
	
		new Subscriber(SensorChannel.ENCODER.getMsgPath(), new MessageListener() 
		{
			@Override
			public void actionPerformed(String topicName, Message m) 
			{
				//System.out.println(m.toLogString());
			}
		});
	}
=======
        
        Gui.EnableLogging();
        SensorManager sm = SensorManager.getInstance();
        
        //initialize a new real sensor with type, port, and channel(s)
        //sensormanager will (eventually) continue to look on same port for the sensor
        //returns a key to the new sensor -- remove with this key.
        String ImuKey = sm.newRealSensor(RealNodeEnum.IMU, config.COM_PORT_IMU, SensorChannel.IMU);
        String GpsKey = sm.newRealSensor(RealNodeEnum.GPS, config.COM_PORT_GPS_INTEGRATED, SensorChannel.GPS);
        String RBSMKey = sm.newRealSensor(RealNodeEnum.RBSM, config.COM_PORT_ENCODER, SensorChannel.ENCODER, SensorChannel.STEERING);
    }
>>>>>>> master
}
