package com.roboclub.robobuggy.nodes.sensors;

import com.hcrest.jfreespace.Device;
import com.hcrest.jfreespace.DeviceListenerInterface;
import com.hcrest.jfreespace.Discovery;
import com.hcrest.jfreespace.DiscoveryListenerInterface;
import com.hcrest.jfreespace.inreport.FreespaceMsgInMotionEngineOutput;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.hcrest.jfreespace.outreport.FreespaceMsgOutDataModeControlV2Request;
import com.hcrest.jfreespace.outreport.HidOutMsg;
import com.roboclub.robobuggy.messages.IMUAccelerationMessage;
import com.roboclub.robobuggy.messages.IMUAngularPositionMessage;
import com.roboclub.robobuggy.messages.IMUAngularVelocityMessage;
import com.roboclub.robobuggy.messages.IMUInclinationMessage;
import com.roboclub.robobuggy.messages.IMULinearAccelerationMessage;
import com.roboclub.robobuggy.messages.IMUCompassMessage;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

/**
 * Driver for the Hillcrest IMUs
 *
 * @author Trevor Decker
 * @author Sean Buckley
 */
public class HillCrestImuNode implements DiscoveryListenerInterface, DeviceListenerInterface, com.roboclub.robobuggy.ros.Node {
    private Device thisDevice;

    private Publisher accelPub = new Publisher(NodeChannel.IMU_ACCELERATION.getMsgPath());
    private Publisher linearAccPub = new Publisher(NodeChannel.IMU_LINEAR_ACC.getMsgPath());
    private Publisher inclinationPub = new Publisher(NodeChannel.IMU_INCLINATION.getMsgPath());
    private Publisher angVelPub = new Publisher(NodeChannel.IMU_ANG_VEL.getMsgPath());
    private Publisher magPub = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());
    private Publisher compassPub = new Publisher(NodeChannel.IMU_COMPASS.getMsgPath());
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
        if (!(arg1 instanceof FreespaceMsgInMotionEngineOutput)) {
            return;
        }

        FreespaceMsgInMotionEngineOutput m = (FreespaceMsgInMotionEngineOutput) arg1;
        int[] data = m.getMeData();
        //assuming message is of type 0
        int offset = 0;


        int axisVal;
        float scale;

        if (offset < 0) {
            return; // Compass heading flag not set
        }


        // FF0 is acceleration for Format 1
        // reported in units of 0.01g
        if (m.getFf0()) {
            int xAccelInHundredths = extractByteFromDataArray(data, offset);
            double xAccel = xAccelInHundredths / 100.0;
            offset += 2;

            int yAccelInHundredths = extractByteFromDataArray(data, offset);
            double yAccel = yAccelInHundredths / 100.0;
            offset += 2;

            int zAccelInHundredths = extractByteFromDataArray(data, offset);
            double zAccel = zAccelInHundredths / 100.0;
            offset += 2;

            accelPub.publish(new IMUAccelerationMessage(xAccel, yAccel, zAccel));

        }
        //ff1 is linear acceleration for Format 1
        // reported in units of 0.01g
        if (m.getFf1()) {
            int xAccelInHundredths = extractByteFromDataArray(data, offset);
            double xAccel = xAccelInHundredths / 100.0;
            offset += 2;

            int yAccelInHundredths = extractByteFromDataArray(data, offset);
            double yAccel = yAccelInHundredths / 100.0;
            offset += 2;

            int zAccelInHundredths = extractByteFromDataArray(data, offset);
            double zAccel = zAccelInHundredths / 100.0;
            offset += 2;

            linearAccPub.publish(new IMULinearAccelerationMessage(xAccel, yAccel, zAccel));
        }
        //ff2 is angular velocity for Format 1
        // reported in units of 0.1 deg/sec
        if (m.getFf2()) {
            int xAngularVelocityInTenths = extractByteFromDataArray(data, offset);
            double xAngularVel = xAngularVelocityInTenths / 10.0;
            offset += 2;

            int yAngularVelocityInTenths = extractByteFromDataArray(data, offset);
            double yAngularVel = yAngularVelocityInTenths / 10.0;
            offset += 2;

            int zAngularVelocityInTenths = extractByteFromDataArray(data, offset);
            double zAngularVel = zAngularVelocityInTenths / 10.0;
            offset += 2;

            angVelPub.publish(new IMUAngularVelocityMessage(xAngularVel, yAngularVel, zAngularVel));
        }
        //ff3 is magnetometer for Format 1
        // reported in units of 0.001 gauss
        if (m.getFf3()) {
            int xMagInThousandths = extractByteFromDataArray(data, offset);
            double xMag = xMagInThousandths / 1000.0;
            offset += 2;

            int yMagInThousandths = extractByteFromDataArray(data, offset);
            double yMag = yMagInThousandths / 1000.0;
            offset += 2;

            int zMagInThousandths = extractByteFromDataArray(data, offset);
            double zMag = zMagInThousandths / 1000.0;
            offset += 2;

            magPub.publish(new MagneticMeasurement(xMag, yMag, zMag));
        }
        //ff4 is inclination
        // reported in units of 0.1 degrees
        if (m.getFf4()) {
            int xInclinationInTenths = extractByteFromDataArray(data, offset);
            double xInclination = xInclinationInTenths / 10.0;
            offset += 2;

            int yInclinationInTenths = extractByteFromDataArray(data, offset);
            double yInclination = yInclinationInTenths / 10.0;
            offset += 2;

            int zInclinationInTenths = extractByteFromDataArray(data, offset);
            double zInclination = zInclinationInTenths / 10.0;
            offset += 2;

            inclinationPub.publish(new IMUInclinationMessage(xInclination, yInclination, zInclination));
        }
        //ff5 is compass for format 1
        if (m.getFf5()) {
            int degreesInTenths = extractByteFromDataArray(data, offset);
            double degrees = degreesInTenths/10.0;
            offset += 2;

            compassPub.publish(new IMUCompassMessage(degrees));
        }
        //ff6 is angular position
        if (m.getFf6()) {
            double w = convertQNToDouble((byte) data[offset + 0], (byte) data[offset + 1], 14);
            double x = convertQNToDouble((byte) data[offset + 2], (byte) data[offset + 3], 14);
            double y = convertQNToDouble((byte) data[offset + 4], (byte) data[offset + 5], 14);
            double z = convertQNToDouble((byte) data[offset + 6], (byte) data[offset + 7], 14);

            double r11 = 1 - 2 * y * y - 2 * z * z;
            double r12 = 2 * x * y - 2 * z * w;
            double r13 = 2 * x * z + 2 * y * w;
            double r21 = 2 * x * y + 2 * z * w;
            double r22 = 1 - 2 * x * x - 2 * z * z;
            double r23 = 2 * y * z - 2 * x * w;
            double r31 = 2 * x * z - 2 * y * w;
            double r32 = 2 * y * z + 2 * x * w;
            double r33 = 1 - 2 * x * x - 2 * y * y;

            //TODO normalize
            //extracts
            double[][] rot = {
                    { r11, r12, r13 },
                    { r21, r22, r23 },
                    { r31, r32, r33 }
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
     *
     * @param lsb least significant byte
     * @param msb most significant byte
     * @param qn  number of fraction bits
     * @return Converted floating point val
     */
    public double convertQNToDouble(byte lsb, byte msb, int qn) {
        // converting Qn fixed point

        int msbAsInt = ((int) msb) & 0xFF;
        int lsbAsInt = ((int) lsb & 0xFF);

        int squashedTogether = ((msbAsInt << 8) | lsbAsInt);
        Integer sT = Integer.valueOf(squashedTogether);

        if (sT >= 1 << 15) {
            sT = sT - (1 << 17 - 1);
        }

        double d = sT.doubleValue();
        return Math.pow(2, -1 * qn) * d;

    }

    private int extractByteFromDataArray(int[] data, int offset) {

        byte lsbByte = (byte) data[offset];
        byte msbByte = (byte) data[offset + 1];
        int lsb = ((int) lsbByte) & 0xFF;
        int msb = ((int) msbByte) & 0xFF;

        return ((msb << 8) | lsb) & 0xFFFF;
    }

}
