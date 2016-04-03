package com.roboclub.robobuggy.nodes.sensors;

import com.hcrest.jfreespace.Device;

import com.hcrest.jfreespace.DeviceListenerInterface;
import com.hcrest.jfreespace.Discovery;
import com.hcrest.jfreespace.DiscoveryListenerInterface;
import com.hcrest.jfreespace.inreport.FreespaceMsgInMotionEngineOutput;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.hcrest.jfreespace.outreport.FreespaceMsgOutDataModeControlV2Request;
import com.hcrest.jfreespace.outreport.HidOutMsg;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.messages.IMULinearAccelerationMessage;
import com.roboclub.robobuggy.messages.IMUAngularVelocityMessage;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.messages.IMUTemperatureMessage;
import com.roboclub.robobuggy.messages.IMUAngularPositionMessage;

/**
 * Driver for the Hillcrest IMUs
 * @author Trevor Decker
 * @author Sean Buckley
 *
 */
public class HillCrestImuNode implements DiscoveryListenerInterface,DeviceListenerInterface, com.roboclub.robobuggy.ros.Node{
	private Device thisDevice;
	private Publisher pub = new Publisher(NodeChannel.HILL_CREST_IMU.getMsgPath());
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
		
		System.out.println("hillcrest enabled");
		
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
		//we do not parse ff0
		if(m.getFf0()){
			offset += 6;
		}
		//ff1 is linear acceleration 
		if(m.getFf1()){
			double xAccel = convertQNToDouble((byte)data[offset+0],(byte)data[offset+1],10);
			double yAccel = convertQNToDouble((byte)data[offset+2],(byte)data[offset+3],10);
			double zAccel = convertQNToDouble((byte)data[offset+4],(byte)data[offset+5],10);
			//System.out.println("xAccel:"+xAccel+"\tyAccel:"+yAccel+"\tzAccel:"+zAccel);
			offset +=6;
			linearAccPub.publish(new IMULinearAccelerationMessage(xAccel, yAccel, zAccel));
		}
		//ff2 is linear Acceleration no gravity
		if(m.getFf2()){
			double xAccel = convertQNToDouble((byte)data[offset+0],(byte)data[offset+1],10);
			double yAccel = convertQNToDouble((byte)data[offset+2],(byte)data[offset+3],10);
			double zAccel = convertQNToDouble((byte)data[offset+4],(byte)data[offset+5],10);
			offset += 6;
//			System.out.println("xAccel:"+xAccel+"\tyAccel:"+yAccel+"\tzAccel:"+zAccel);
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
			
			//TODO normalize 
			//extracts 
			double[][] rot = {
				{1-2*y*y-2*z*z,      2*x*y-2*z*w,             2*x*z+2*y*w},
				{2*x*y+2*z*w,        1-2*x*x-2*z*z,           2*y*z-2*x*w},
				{2*x*z-2*y*w,        2*y*z+2*x*w,            1-2*x*x-2*y*y}
			};

			angPosPub.publish(new IMUAngularPositionMessage(rot));
			offset += 8;
		}
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
	msg.setFormatSelect(0);
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
		
		//boolean sign = ((int)msb>>7) == 1;
		int msbAsInt = ((int)msb) & 0xFF;//0x7F;
		int lsbAsInt = ((int)lsb & 0xFF);
		
		int squashedTogether = ((msbAsInt << 8) | lsbAsInt);
		Integer sT = new Integer(squashedTogether);
/*
		if(sign){
			sT = -sT;
		}
		*/
		
		if(sT >= 1<<15){
			sT = sT - (1<<17-1);
		}
		
		double d = sT.doubleValue();
		return Math.pow(2, -1 * qn) * d;
		
	}
	
}
