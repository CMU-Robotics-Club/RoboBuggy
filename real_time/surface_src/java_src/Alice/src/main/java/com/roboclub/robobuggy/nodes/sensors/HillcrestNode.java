package com.roboclub.robobuggy.nodes.sensors;

import com.hcrest.jfreespace.Device;
import com.hcrest.jfreespace.DeviceListenerInterface;
import com.hcrest.jfreespace.Discovery;
import com.hcrest.jfreespace.DiscoveryListenerInterface;
import com.hcrest.jfreespace.FreespaceErrorCodes;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.hcrest.jfreespace.outreport.FreespaceMsgOutDataModeControlV2Request;
import com.hcrest.jfreespace.outreport.HidOutMsg;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * A node to communicate with the Hillcrest Freespace 9DOF IMU
 */
public class HillcrestNode extends BuggyDecoratorNode implements DeviceListenerInterface, DiscoveryListenerInterface {

    private Device hillcrestImu;

    /**
     * Creates a new Hillcrest IMU node
     */
    public HillcrestNode() {
        super(new BuggyBaseNode(NodeChannel.IMU), "Hillcrest IMU");
    }

    @Override
    protected boolean startDecoratorNode() {
        // look for an IMU, and add this as a listener
        Discovery.getInstance().addListener(this);

        return true;
    }

    @Override
    protected boolean shutdownDecoratorNode() {
        Discovery.getInstance().removeListener(this);
        return true;
    }

    @Override
    public void handleRead(Device device, HidInMsg hidInMsg, int i, long l) {

    }

    @Override
    public void handleSend(Device device, HidOutMsg hidOutMsg, int i) {

    }

    @Override
    public void notifyRemoved(Device device) {

    }

    @Override
    public void freespaceDeviceInserted(Device device) {
        hillcrestImu = device;
        hillcrestImu.open(this);

        int modeAndStatus = 8; // bit 3 set gives Full Motion On mode - page 41
        int packetSelect  = 8; // 8 = MotionEngineOutput              - page 41
        int formatSelect  = 1; // Mode 1                              - page 18
        FreespaceMsgOutDataModeControlV2Request configMsg = new FreespaceMsgOutDataModeControlV2Request();

        configMsg.setModeAndStatus(modeAndStatus);
        configMsg.setPacketSelect(packetSelect);
        configMsg.setFormatSelect(formatSelect);

        // get all the data
        configMsg.setFf0(true);
        configMsg.setFf1(true);
        configMsg.setFf2(true);
        configMsg.setFf3(true);
        configMsg.setFf4(true);
        configMsg.setFf5(true);
        configMsg.setFf6(true);
        configMsg.setFf7(true);

        FreespaceErrorCodes response = hillcrestImu.sendMessage(configMsg);
        if (!response.equals(FreespaceErrorCodes.FREESPACE_SUCCESS)) {
            new RobobuggyLogicNotification("Error configuring IMU!", RobobuggyMessageLevel.EXCEPTION);
        }
    }

    @Override
    public void freespaceDeviceRemoved(Device device) {
        hillcrestImu.close();
        hillcrestImu = null;
    }
}
