package com.roboclub.robobuggy.nodes.sensors;

import com.hcrest.jfreespace.Device;
import com.hcrest.jfreespace.DeviceListenerInterface;
import com.hcrest.jfreespace.Discovery;
import com.hcrest.jfreespace.DiscoveryListenerInterface;
import com.hcrest.jfreespace.FreespaceErrorCodes;
import com.hcrest.jfreespace.inreport.FreespaceMsgInMotionEngineOutput;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.hcrest.jfreespace.outreport.FreespaceMsgOutDataModeControlV2Request;
import com.hcrest.jfreespace.outreport.HidOutMsg;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.IMUAccelerationMessage;
import com.roboclub.robobuggy.messages.IMUAngularPositionMessage;
import com.roboclub.robobuggy.messages.IMUAngularVelocityMessage;
import com.roboclub.robobuggy.messages.IMUCompassMessage;
import com.roboclub.robobuggy.messages.IMUInclinationMessage;
import com.roboclub.robobuggy.messages.IMULinearAccelerationMessage;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

/**
 * A node to communicate with the Hillcrest Freespace 9DOF IMU
 *
 * Comments will make callbacks to page numbers on the datasheet/reference manual, which can be found here:
 * http://hillcrestlabs.com/resources/download-materials/download-info/hcomm-reference-manual/
 */
@SuppressWarnings("Duplicates")
public class HillcrestNode extends BuggyDecoratorNode implements DeviceListenerInterface, DiscoveryListenerInterface {

    private Device hillcrestImu;

    // Constants
    private static final int IMU_FORMAT_MODE = 1; // Mode 1 - page 21

    // Publishers for the different messages
    private Publisher accelerationPub;
    private Publisher linearAccelerationPub;
    private Publisher angularVelocityPub;
    private Publisher magnetometerPub;
    private Publisher inclinationPub;
    private Publisher compassHeadingPub;
    private Publisher angularPositionPub;

    /**
     * Creates a new Hillcrest IMU node
     */
    public HillcrestNode() {
        super(new BuggyBaseNode(NodeChannel.IMU), "Hillcrest IMU");

        // initialize the publishers
        accelerationPub = new Publisher(NodeChannel.IMU_ACCELERATION.getMsgPath());
        linearAccelerationPub = new Publisher(NodeChannel.IMU_LINEAR_ACC.getMsgPath());
        angularVelocityPub = new Publisher(NodeChannel.IMU_ANG_VEL.getMsgPath());
        magnetometerPub = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());
        inclinationPub = new Publisher(NodeChannel.IMU_INCLINATION.getMsgPath());
        compassHeadingPub = new Publisher(NodeChannel.IMU_COMPASS.getMsgPath());
        angularPositionPub = new Publisher(NodeChannel.IMU_ANG_POS.getMsgPath());
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

    /**
     * This method parses the IMU data array by using the spec on Pages 22 and 23 of the
     * reference manual
     *
     * @param device the IMU in question
     * @param message the message from the IMU
     * @param i [Unknown]
     * @param l [Unknown]
     */
    @Override
    public void handleRead(Device device, HidInMsg message, int i, long l) {

        // first check whether the message was the kind we're looking for
        // at this point we are only handling MotionEngineOutput messages
        if (!(message instanceof FreespaceMsgInMotionEngineOutput)) {
            new RobobuggyLogicNotification("IMU gave us message other than motion engine output!", RobobuggyMessageLevel.NOTE);
            return;
        }

        FreespaceMsgInMotionEngineOutput imuMotionEngineOutput = ((FreespaceMsgInMotionEngineOutput) message);

        // check whether we are in the correct mode
        if (imuMotionEngineOutput.getFormatSelect() != IMU_FORMAT_MODE) {
            new RobobuggyLogicNotification("IMU is in an unexpected mode!", RobobuggyMessageLevel.EXCEPTION);
            return;
        }

        // get the data array, and start our tracker
        int[] imuDataArray = imuMotionEngineOutput.getMeData();
        int dataArrayOffset = 0;

        // FF0 measures acceleration in the x, y, z coordinates (outlined on the device)
        //  x positive is to the right
        //  y positive is forward
        //  z positive is up
        // reported in units of 0.01g
        if (imuMotionEngineOutput.getFf0()) {

            int xAccelInHundredths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double xAccel = xAccelInHundredths / 100.0;
            dataArrayOffset += 2;

            int yAccelInHundredths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double yAccel = yAccelInHundredths / 100.0;
            dataArrayOffset += 2;

            int zAccelInHundredths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double zAccel = zAccelInHundredths / 100.0;
            dataArrayOffset += 2;

            accelerationPub.publish(new IMUAccelerationMessage(xAccel, yAccel, zAccel));

        }

        // FF1 measures linear acceleration in the xyz plane (outlined on the device)
        //  x positive is right
        //  y positive is forward
        //  z positive is up
        // reported in units of 0.01g
        if (imuMotionEngineOutput.getFf1()) {

            int xAccelInHundredths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double xAccel = xAccelInHundredths / 100.0;
            dataArrayOffset += 2;

            int yAccelInHundredths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double yAccel = yAccelInHundredths / 100.0;
            dataArrayOffset += 2;

            int zAccelInHundredths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double zAccel = zAccelInHundredths / 100.0;
            dataArrayOffset += 2;

            linearAccelerationPub.publish(new IMULinearAccelerationMessage(xAccel, yAccel, zAccel));

        }

        // FF2 measures angular velocity
        //  +x is tilt up (pitch)
        //  +y is tilt right (roll)
        //  +z is turn left (yaw)
        // reported in units of 0.1 degrees/sec
        if (imuMotionEngineOutput.getFf2()) {

            int xAngularVelocityInTenths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double xAngularVel = xAngularVelocityInTenths / 10.0;
            dataArrayOffset += 2;

            int yAngularVelocityInTenths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double yAngularVel = yAngularVelocityInTenths / 10.0;
            dataArrayOffset += 2;

            int zAngularVelocityInTenths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double zAngularVel = zAngularVelocityInTenths / 10.0;
            dataArrayOffset += 2;

            angularVelocityPub.publish(new IMUAngularVelocityMessage(xAngularVel, yAngularVel, zAngularVel));

        }

        // FF3 measures the magnetometer with respect to the device frame of reference
        //  +x is forward
        //  +y is to the right
        //  +z is down
        // reported in units of 0.001 gauss
        if (imuMotionEngineOutput.getFf3()) {

            int xMagInThousandths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double xMag = xMagInThousandths / 1000.0;
            dataArrayOffset += 2;

            int yMagInThousandths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double yMag = yMagInThousandths / 1000.0;
            dataArrayOffset += 2;

            int zMagInThousandths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double zMag = zMagInThousandths / 1000.0;
            dataArrayOffset += 2;

            magnetometerPub.publish(new MagneticMeasurement(xMag, yMag, zMag));

        }

        // FF4 measures the inclination
        //  +x is tilt up (pitch)
        //  +y is tilt right (roll)
        //  +z is turn left (yaw)
        // reported in units of 0.1 degrees
        if (imuMotionEngineOutput.getFf4()) {

            int xInclinationInTenths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double xInclination = xInclinationInTenths / 10.0;
            dataArrayOffset += 2;

            int yInclinationInTenths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double yInclination = yInclinationInTenths / 10.0;
            dataArrayOffset += 2;

            int zInclinationInTenths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double zInclination = zInclinationInTenths / 10.0;
            dataArrayOffset += 2;

            inclinationPub.publish(new IMUInclinationMessage(xInclination, yInclination, zInclination));

        }

        // FF5 measures the compass heading
        // reported in units of 0.1 degrees
        if (imuMotionEngineOutput.getFf5()) {

            int degreesInTenths = extractHalfWordFromDataArray(imuDataArray, dataArrayOffset);
            double degrees = degreesInTenths/10.0;
            dataArrayOffset += 2;

            compassHeadingPub.publish(new IMUCompassMessage(degrees));

        }

        // FF6 measures angular position
        //  Gives 4 quaternions (dimensionless) that are expressed as Q14 decimals
        if (imuMotionEngineOutput.getFf6()) {

            double w = convertQNToDouble(extractHalfWordFromDataArray(imuDataArray, dataArrayOffset), 14);
            dataArrayOffset += 2;
            double x = convertQNToDouble(extractHalfWordFromDataArray(imuDataArray, dataArrayOffset), 14);
            dataArrayOffset += 2;
            double y = convertQNToDouble(extractHalfWordFromDataArray(imuDataArray, dataArrayOffset), 14);
            dataArrayOffset += 2;
            double z = convertQNToDouble(extractHalfWordFromDataArray(imuDataArray, dataArrayOffset), 14);
            dataArrayOffset += 2;

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

            angularPositionPub.publish(new IMUAngularPositionMessage(rot));

        }

    }

    /**
     * Returns a 2-byte data value from the IMU's data array
     * @param data the byte array for the IMU
     * @param offsetInBytes the offset into the data array where the halfword starts from. Note that this value does
     *                      NOT get increased, you will need to keep track of an offset externally
     * @return the halfword extracted
     */
    protected int extractHalfWordFromDataArray(int[] data, int offsetInBytes) {

        int lsb = data[offsetInBytes];
        int msb = data[offsetInBytes + 1];
        return ((msb << 8) | lsb);

    }


    /**
     * Converts from fixed point to floating point
     *
     * Some of the imu data is expressed as fixed point numbers, with the format of Q{N},
     * where Q represents 'fixed point', and N represents the point. Q10 represents a
     * fixed point number with 1 sign bit, 5 integer bits and 10 fractional bits
     *
     * More info is found on Page 18
     *
     * The algorithm is as follows on Wikipedia:
     * https://en.wikipedia.org/wiki/Q_(number_format)#Q_to_float
     *
     * @param q the 16-bit fixed-point number
     * @param n the location of the point
     * @return Converted floating point val
     */
    protected double convertQNToDouble(int q, int n) {

        // convert it to a double
        double qDouble = (double) q;

        // get the sign bit
        int signBit = (q >> 15) & 0x1;
        int sign = 0;
        if (signBit == 0) {
            sign = 1;
        }
        else {
            sign = -1;
        }

        // multiply it by 2^(-n)
        return sign * qDouble * Math.pow(2, -1 * n);

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

        int modeAndStatus = 8;               // bit 3 set gives Full Motion On mode - page 41
        int packetSelect  = 8;               // 8 = MotionEngineOutput              - page 41
        int formatSelect  = IMU_FORMAT_MODE; // Mode 1                              - page 21

        FreespaceMsgOutDataModeControlV2Request configMsg = new FreespaceMsgOutDataModeControlV2Request();

        configMsg.setModeAndStatus(modeAndStatus);
        configMsg.setPacketSelect(packetSelect);
        configMsg.setFormatSelect(formatSelect);

        // we want to make sure that we get all the packets
        // the Format Flags are basically a way to enable each individual sensor - page 41
        configMsg.setFf0(true);
        configMsg.setFf1(true);
        configMsg.setFf2(true);
        configMsg.setFf3(true);
        configMsg.setFf4(true);
        configMsg.setFf5(true);
        configMsg.setFf6(true);
        configMsg.setFf7(true);

        // send down the config info, and get the response
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
