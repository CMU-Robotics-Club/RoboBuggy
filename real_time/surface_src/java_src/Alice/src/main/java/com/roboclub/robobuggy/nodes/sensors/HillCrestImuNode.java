package com.roboclub.robobuggy.nodes.sensors;


import com.hcrest.jfreespace.Device;
import com.hcrest.jfreespace.DeviceListenerInterface;
import com.hcrest.jfreespace.DeviceStatistics;
import com.hcrest.jfreespace.Discovery;
import com.hcrest.jfreespace.DiscoveryListenerInterface;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.hcrest.jfreespace.outreport.FreespaceMsgOutDataModeControlV2Request;
import com.hcrest.jfreespace.outreport.HidOutMsg;
import com.roboclub.robobuggy.messages.HillCrestIMUMessage;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import boofcv.alg.segmentation.ms.MergeSmallRegions.Node;

public class HillCrestImuNode implements DiscoveryListenerInterface,DeviceListenerInterface, com.roboclub.robobuggy.ros.Node{
	private Device device_;
	private DeviceStatistics deviceStats;
	private Publisher pub = new Publisher(NodeChannel.HILL_CREST_IMU.getMsgPath());
	
	public HillCrestImuNode() {
		super();
		Discovery discover = Discovery.getInstance();
		discover.addListener(this);
		
		// TODO Auto-generated constructor stub
	}

	@Override
	public void handleRead(Device arg0, HidInMsg arg1, int arg2, long arg3) {
		// TODO Auto-generated method stub
		System.out.println(arg1.toDetailedString());
		pub.publish(new HillCrestIMUMessage(arg1.toDetailedString()));
		
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
		device_ = arg0;
		device_.open(this);
		
	deviceStats = device_.getStatistics();
	System.out.println("found device");
	System.out.println(deviceStats.toString());
	FreespaceMsgOutDataModeControlV2Request msg = new FreespaceMsgOutDataModeControlV2Request();
	msg.setPacketSelect(8);  //
	msg.setModeAndStatus(0);
	msg.setFormatSelect(0);
	msg.setFf0(true);
	msg.setFf1(true);
	msg.setFf2(true);
	msg.setFf3(true);
	msg.setFf4(true);
	msg.setFf5(true);
	msg.setFf6(true);
	msg.setFf7(true);

	device_.sendMessageAsync(msg);
	System.out.println("setup read");
	}

	@Override
	public void freespaceDeviceRemoved(Device arg0) {
		// TODO Auto-generated method stub
		device_ = null;

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
	
	
	
}
