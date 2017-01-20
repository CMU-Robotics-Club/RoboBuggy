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

    }


    @Override
    public void handleRead(Device arg0, HidInMsg arg1, int arg2, long arg3) {
        //TODO handle other kinds of messages
        if (!(arg1 instanceof FreespaceMsgInMotionEngineOutput)) {
            return;
        }

        FreespaceMsgInMotionEngineOutput m = (FreespaceMsgInMotionEngineOutput) arg1;

        // we can only parse format 1 for now
        if (m.getFormatSelect() != 1) {
            return;
        }

        int[] data = m.getMeData();
        int offsetInBytes = 0;

        // FF0 is acceleration for Format 1
        // reported in units of 0.01g
        if (m.getFf0()) {
            int xAccelInHundredths = extractHalfWordFromDataArray(data, offsetInBytes);
            double xAccel = xAccelInHundredths / 100.0;
            offsetInBytes += 2;

            int yAccelInHundredths = extractHalfWordFromDataArray(data, offsetInBytes);
            double yAccel = yAccelInHundredths / 100.0;
            offsetInBytes += 2;

            int zAccelInHundredths = extractHalfWordFromDataArray(data, offsetInBytes);
            double zAccel = zAccelInHundredths / 100.0;
            offsetInBytes += 2;

            accelPub.publish(new IMUAccelerationMessage(xAccel, yAccel, zAccel));

        }
        //ff1 is linear acceleration for Format 1
        // reported in units of 0.01g
        if (m.getFf1()) {
            int xAccelInHundredths = extractHalfWordFromDataArray(data, offsetInBytes);
            double xAccel = xAccelInHundredths / 100.0;
            offsetInBytes += 2;

            int yAccelInHundredths = extractHalfWordFromDataArray(data, offsetInBytes);
            double yAccel = yAccelInHundredths / 100.0;
            offsetInBytes += 2;

            int zAccelInHundredths = extractHalfWordFromDataArray(data, offsetInBytes);
            double zAccel = zAccelInHundredths / 100.0;
            offsetInBytes += 2;

            linearAccPub.publish(new IMULinearAccelerationMessage(xAccel, yAccel, zAccel));
        }
        //ff2 is angular velocity for Format 1
        // reported in units of 0.1 deg/sec
        if (m.getFf2()) {
            int xAngularVelocityInTenths = extractHalfWordFromDataArray(data, offsetInBytes);
            double xAngularVel = xAngularVelocityInTenths / 10.0;
            offsetInBytes += 2;

            int yAngularVelocityInTenths = extractHalfWordFromDataArray(data, offsetInBytes);
            double yAngularVel = yAngularVelocityInTenths / 10.0;
            offsetInBytes += 2;

            int zAngularVelocityInTenths = extractHalfWordFromDataArray(data, offsetInBytes);
            double zAngularVel = zAngularVelocityInTenths / 10.0;
            offsetInBytes += 2;

            angVelPub.publish(new IMUAngularVelocityMessage(xAngularVel, yAngularVel, zAngularVel));
        }
        //ff3 is magnetometer for Format 1
        // reported in units of 0.001 gauss
        if (m.getFf3()) {
            int xMagInThousandths = extractHalfWordFromDataArray(data, offsetInBytes);
            double xMag = xMagInThousandths / 1000.0;
            offsetInBytes += 2;

            int yMagInThousandths = extractHalfWordFromDataArray(data, offsetInBytes);
            double yMag = yMagInThousandths / 1000.0;
            offsetInBytes += 2;

            int zMagInThousandths = extractHalfWordFromDataArray(data, offsetInBytes);
            double zMag = zMagInThousandths / 1000.0;
            offsetInBytes += 2;

            magPub.publish(new MagneticMeasurement(xMag, yMag, zMag));
        }
        //ff4 is inclination
        // reported in units of 0.1 degrees
        if (m.getFf4()) {
            int xInclinationInTenths = extractHalfWordFromDataArray(data, offsetInBytes);
            double xInclination = xInclinationInTenths / 10.0;
            offsetInBytes += 2;

            int yInclinationInTenths = extractHalfWordFromDataArray(data, offsetInBytes);
            double yInclination = yInclinationInTenths / 10.0;
            offsetInBytes += 2;

            int zInclinationInTenths = extractHalfWordFromDataArray(data, offsetInBytes);
            double zInclination = zInclinationInTenths / 10.0;
            offsetInBytes += 2;

            inclinationPub.publish(new IMUInclinationMessage(xInclination, yInclination, zInclination));
        }
        //ff5 is compass for format 1
        if (m.getFf5()) {
            int degreesInTenths = extractHalfWordFromDataArray(data, offsetInBytes);
            double degrees = degreesInTenths/10.0;
            offsetInBytes += 2;

            compassPub.publish(new IMUCompassMessage(degrees));
        }
        //ff6 is angular position
        if (m.getFf6()) {
            double w = convertQNToDouble((byte) data[offsetInBytes + 0], (byte) data[offsetInBytes + 1], 14);
            double x = convertQNToDouble((byte) data[offsetInBytes + 2], (byte) data[offsetInBytes + 3], 14);
            double y = convertQNToDouble((byte) data[offsetInBytes + 4], (byte) data[offsetInBytes + 5], 14);
            double z = convertQNToDouble((byte) data[offsetInBytes + 6], (byte) data[offsetInBytes + 7], 14);

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
            offsetInBytes += 8;
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

    /**
     * Returns a 2-byte data value from the IMU's data array
     * @param data the byte array for the IMU
     * @param offsetInBytes the offset into the data array where the halfword starts from. Note that this value does
     *                      NOT get increased, you will need to keep track of an offset externally
     * @return the halfword extracted
     */
    private int extractHalfWordFromDataArray(int[] data, int offsetInBytes) {

        int lsb = data[offsetInBytes];
        int msb = data[offsetInBytes + 1];
        return ((msb << 8) | lsb);
    }

}
