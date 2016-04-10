package com.roboclub.robobuggy.nodes.sensors;

import com.hcrest.jfreespace.Device;
import com.hcrest.jfreespace.DeviceListenerInterface;
import com.hcrest.jfreespace.Discovery;
import com.hcrest.jfreespace.DiscoveryListenerInterface;
import com.hcrest.jfreespace.inreport.FreespaceMsgInMotionEngineOutput;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.hcrest.jfreespace.outreport.FreespaceMsgOutDataModeControlV2Request;
import com.hcrest.jfreespace.outreport.HidOutMsg;
import com.roboclub.robobuggy.messages.IMUAngularPositionMessage;
import com.roboclub.robobuggy.messages.IMUAngularVelocityMessage;
import com.roboclub.robobuggy.messages.IMULinearAccelerationMessage;
import com.roboclub.robobuggy.messages.IMUTemperatureMessage;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

/**
 * Driver for the Hillcrest IMUs
 * @author Trevor Decker
 * @author Sean Buckley
 *
 */
public class HillCrestImuNode implements DiscoveryListenerInterface,DeviceListenerInterface, com.roboclub.robobuggy.ros.Node{
	private Device thisDevice;
	private Publisher linearAccPub = new Publisher(NodeChannel.IMU_LINEAR_ACC.getMsgPath());
	private Publisher linearAccNoGravPub = new Publisher(NodeChannel.IMU_LINEAR_NO_GRAV.getMsgPath());
	private Publisher angVelPub = new Publisher(NodeChannel.IMU_ANG_VEL.getMsgPath());
	private Publisher magPub = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());
	private Publisher tempPub = new Publisher(NodeChannel.IMU_TEMP.getMsgPath());
	private Publisher angPosPub = new Publisher(NodeChannel.IMU_ANG_POS.getMsgPath());
	
	/**
	 * Constructor for the hillcrest imu
	 */
	public HillCrestImuNode() {
		super();
		Discovery discover = Discovery.getInstance();
		discover.addListener(this);
		
		
		// TODO Auto-generated constructor stub
	}

	
	
	@Override
	public void handleRead(Device arg0, HidInMsg arg1, int arg2, long arg3) {
		//TODO handle other kinds of messages
		if (!(arg1 instanceof FreespaceMsgInMotionEngineOutput)){
			return;
		}

		FreespaceMsgInMotionEngineOutput m = (FreespaceMsgInMotionEngineOutput)arg1;
		int[] data = m.getMeData();
		//assuming message is of type 0
		int offset = 0;
		
		
	    int axisVal;
	    float scale;

	    switch(m.getFormatSelect()) {
	    case 1:
	        scale = 10; // 0.1 degrees
	        offset =  offset + 6; // Skip over acc
	        offset = offset + 6;// Skip over lin acc
	        offset = offset + 6; // Skip over ang vel
	        offset = offset + 6; // Skip over mag
	        offset = offset + 6; // Skip over inclination
	        break;
	    case 0:
	    case 2:
	    case 3:
	        return; // No calibrated compass heading in this format
	    default:
	        return; // The format number was unrecognized
	    }

	    if (offset < 0) {
	        return; // Compass heading flag not set
	    }

	    // Extract and convert the compass heading data
	    axisVal = data[offset + 1] << 8 |  data[offset + 0];
	    double compassHeading = convertQNToDouble((byte)data[offset], (byte)data[offset+1], 10);//((float) axisVal) / scale;
		
		System.out.println("Compass heading: " + compassHeading);
	    

		
		/*
		
		//we do not parse ff0
		if(m.getFf0()){
			offset += 6;
		}
		//ff1 is linear acceleration 
		if(m.getFf1()){
			double xAccel = convertQNToDouble((byte)data[offset+0],(byte)data[offset+1],10);
			double yAccel = convertQNToDouble((byte)data[offset+2],(byte)data[offset+3],10);
			double zAccel = convertQNToDouble((byte)data[offset+4],(byte)data[offset+5],10);
			offset +=6;
			linearAccPub.publish(new IMULinearAccelerationMessage(xAccel, yAccel, zAccel));
		}
		//ff2 is linear Acceleration no gravity
		if(m.getFf2()){
			double xAccel = convertQNToDouble((byte)data[offset+0],(byte)data[offset+1],10);
			double yAccel = convertQNToDouble((byte)data[offset+2],(byte)data[offset+3],10);
			double zAccel = convertQNToDouble((byte)data[offset+4],(byte)data[offset+5],10);
			offset += 6;
			linearAccNoGravPub.publish(new IMULinearAccelerationMessage(xAccel, yAccel, zAccel));
		}
		//ff3 is Angular velocity
		if(m.getFf3()){
			double xAngularVel =convertQNToDouble((byte)data[offset+0],(byte)data[offset+1],10);
			double yAngularVel = convertQNToDouble((byte)data[offset+2],(byte)data[offset+3],10);
			double zAngularVel = convertQNToDouble((byte)data[offset+4],(byte)data[offset+5],10);
			offset +=6;
			angVelPub.publish(new IMUAngularVelocityMessage(xAngularVel, yAngularVel, zAngularVel));
		}
		//ff4 is magnetometer
		if(m.getFf4()){
			double xMag =convertQNToDouble((byte)data[offset+0],(byte)data[offset+1],12);
			double yMag =convertQNToDouble((byte)data[offset+2],(byte)data[offset+3],12);
			double zMag =convertQNToDouble((byte)data[offset+4],(byte)data[offset+5],12);
			offset += 6;
			magPub.publish(new MagneticMeasurement(xMag, yMag, zMag));
		}
		//ff5 is temperature
		if(m.getFf5()){
			double temperature = convertQNToDouble((byte)data[offset+0],(byte)data[offset+1],7);
			offset += 2;
			tempPub.publish(new IMUTemperatureMessage(temperature));
		}
		//ff6 is angular position
		if(m.getFf6()){
			double w = convertQNToDouble((byte)data[offset+0],(byte)data[offset+1],14);
			double x = convertQNToDouble((byte)data[offset+2],(byte)data[offset+3],14);
			double y = convertQNToDouble((byte)data[offset+4],(byte)data[offset+5],14);
			double z = convertQNToDouble((byte)data[offset+6],(byte)data[offset+7],14);
			
			double r11 = 1-2*y*y-2*z*z;
			double r12 = 2*x*y-2*z*w;
			double r13 = 2*x*z+2*y*w;
			double r21 = 2*x*y+2*z*w;
			double r22 = 1-2*x*x-2*z*z;
			double r23 = 2*y*z-2*x*w;
			double r31 = 2*x*z-2*y*w;
			double r32 = 2*y*z+2*x*w;
			double r33 = 1-2*x*x-2*y*y;
			
			//TODO normalize 
			//extracts 
			double[][] rot = {
				{r11, r12, r13},
				{r21, r22, r23},
				{r31, r32, r33}
			};
			
			double yaw = Math.atan(r21/r11);
			double pitch = -r31/Math.sqrt(r32*r32 + r33*r33);
			double roll = Math.atan(r32/r33);
			
			System.out.println("Yaw: " + yaw + " Pitch: " + pitch + " Roll: " + roll);
			
			angPosPub.publish(new IMUAngularPositionMessage(rot));
			offset += 8;
		}
		*/
	}

	@Override
	public void handleSend(Device arg0, HidOutMsg arg1, int arg2) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void notifyRemoved(Device arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void freespaceDeviceInserted(Device arg0) {
		// TODO Auto-generated method stub
		thisDevice = arg0;
		thisDevice.open(this);
		
	FreespaceMsgOutDataModeControlV2Request msg = new FreespaceMsgOutDataModeControlV2Request();
	msg.setPacketSelect(8);  //
	msg.setModeAndStatus(8);
	//Set to 0 for gyro, accel, temp, mag, ang data.  Set to 1 for compass
	msg.setFormatSelect(1);
	//we really don't know what the format is so we are just going to log everything for now 
	msg.setFf2(true); //linear Acceleration without Gravity 
	msg.setFf3(true); //Angular Velocity
	msg.setFf4(true); //Magnetometer
	msg.setFf5(true); //Tempeature
	msg.setFf6(true); //Angular position

	thisDevice.sendMessageAsync(msg);
	}

	@Override
	public void freespaceDeviceRemoved(Device arg0) {
		// TODO Auto-generated method stub
		thisDevice = null;

	}

	@Override
	public boolean startNode() {
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	public boolean shutdown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void setName(String newName) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return "hillcrestImu";
	}
	
	/**
	 * Func: Converts from fixed point to floating point
	 * @param lsb least significant byte
	 * @param msb most significant byte
	 * @param qn number of fraction bits
	 * @return Converted floating point val
	 */
	public double convertQNToDouble(byte lsb, byte msb,int qn) {
		// converting Qn fixed point
		
		int msbAsInt = ((int)msb) & 0xFF;
		int lsbAsInt = ((int)lsb & 0xFF);
		
		int squashedTogether = ((msbAsInt << 8) | lsbAsInt);
		Integer sT = Integer.valueOf(squashedTogether);
		
		if(sT >= 1<<15){
			sT = sT - (1<<17-1);
		}
		
		double d = sT.doubleValue();
		return Math.pow(2, -1 * qn) * d;
		
	}
	
}
