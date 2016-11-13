package com.roboclub.robobuggy.nodes.localizers;

import com.hcrest.jfreespace.Device;
import com.hcrest.jfreespace.inreport.DecoderFactory;
import com.hcrest.jfreespace.inreport.FreespaceMsgInMotionEngineOutput;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.roboclub.robobuggy.messages.*;
import com.roboclub.robobuggy.nodes.sensors.HillCrestImuNode;
import com.roboclub.robobuggy.ros.*;

/**
 * Created by Alex on 11/10/2016.
 */
public class MultipleIMUtest {
    private Subscriber accSub = new Subscriber("test", NodeChannel.IMU_ACCELERATION.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUAccelerationMessage){
            System.out.println("Acceleration Message; Topic is : " + topicName);
            IMUAccelerationMessage mes = ((IMUAccelerationMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getXAccel());
            System.out.println("Y: " + mes.getYAccel());
            System.out.println("Z: " + mes.getZAccel());
            System.out.println("\n====================================================");
        }
    });

    private Subscriber linAccSub = new Subscriber("test", NodeChannel.IMU_LINEAR_ACC.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMULinearAccelerationMessage){
            System.out.println("Linear Acceleration Message; Topic is : " + topicName);
            IMULinearAccelerationMessage mes = ((IMULinearAccelerationMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getX());
            System.out.println("Y: " + mes.getY());
            System.out.println("Z: " + mes.getZ());
            System.out.println("\n====================================================");
        }
    });

    private Subscriber angVelSub = new Subscriber("test", NodeChannel.IMU_ANG_VEL.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUAngularVelocityMessage){
            System.out.println("Angular Velocity Message; Topic is : " + topicName);
            IMUAngularVelocityMessage mes = ((IMUAngularVelocityMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getX());
            System.out.println("Y: " + mes.getY());
            System.out.println("Z: " + mes.getZ());
            System.out.println("\n====================================================");
        }
    });

    private Subscriber magSub = new Subscriber("test", NodeChannel.IMU_MAGNETIC.getMsgPath(), (topicName, m) -> {
        if(m instanceof MagneticMeasurement){
            System.out.println("Magnetic Measurement Message; Topic is : " + topicName);
            MagneticMeasurement mes = ((MagneticMeasurement)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getmagX());
            System.out.println("Y: " + mes.getmagY());
            System.out.println("Z: " + mes.getmagZ());
            System.out.println("\n====================================================");
        }
    });

    private Subscriber incSub = new Subscriber("test", NodeChannel.IMU_INCLINATION.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUInclinationMessage){
            System.out.println("Inclination Message; Topic is : " + topicName);
            IMUInclinationMessage mes = ((IMUInclinationMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getXInclination());
            System.out.println("Y: " + mes.getYInclination());
            System.out.println("Z: " + mes.getZInclination());
            System.out.println("\n====================================================");
        }
    });

    private Subscriber compSub = new Subscriber("test", NodeChannel.IMU_COMPASS.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUCompassMessage){
            System.out.println("Compass Message; Topic is : " + topicName);
            IMUCompassMessage mes = ((IMUCompassMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("Heading: " + mes.getCompassHeading());
            System.out.println("\n====================================================");
        }
    });

    private Subscriber angPosSub = new Subscriber("test", NodeChannel.IMU_ANG_POS.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUAngularPositionMessage){
            System.out.println("Angular Position Message; Topic is : " + topicName);
            IMUAngularPositionMessage mes = ((IMUAngularPositionMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            double[][] rot = mes.getRot();
            System.out.println("Values");
            System.out.println(rot.toString());
            System.out.println("\n====================================================");
        }
    });

    public static void main() {
        fakeHillcrest a = new fakeHillcrest(1, "HillCrest A", 1, 1, 1);
        fakeHillcrest b = new fakeHillcrest(2, "HillCrest B", 1, 1, 1);

        HillCrestImuNode node = new HillCrestImuNode();

        node.freespaceDeviceInserted(a);
        node.freespaceDeviceInserted(b);

        HidInMsg msgA = makeMessage(0);
        HidInMsg msgB = makeMessage(1);

        node.handleRead(a, msgA, 34, System.currentTimeMillis());
        node.handleRead(b, msgA, 34, System.currentTimeMillis());
        node.handleRead(a, msgB, 34, System.currentTimeMillis());
        node.handleRead(b, msgB, 34, System.currentTimeMillis());
    }

    private static HidInMsg makeMessage(int id) {

        boolean a = id == 0;

        byte[] data = new byte[40];

        /** Acceleration **/

        data[0] = 0x20; /** X - Value: 8 **/
        data[1] = 0x03;
        data[2] = 0x50; /** Y - Value: 3.36 **/
        data[3] = 0x01;
        data[4] = 0x00; /** Z - Value: 40.96 for id = 0, 5.12 otherwise **/
        data[5] = (byte) (a ? 0x10 : 0x02);

        /** Linear Acceleration **/

        data[6] = 0x50; /** X - Value: 4 **/
        data[7] = 0x01;
        data[8] = 0x20; /** Y - Value: 8 **/
        data[9] = 0x03;
        data[10] = 0x00; /** Z - Value: 3 for id = 0, 0.44 otherwise **/
        data[11] = (byte) (a ? 0x01 : 0x00);

        /** Angular Velocity **/

        data[12] = 0x55; /** X - Value: 8.5 **/
        data[13] = 0x00;
        data[14] = 0x5F; /** Y - Value: 9.5 **/
        data[15] = 0x00;
        data[16] = 0x7E; /** Z - Value: 63.8 if id = 0, 115 otherwise **/
        data[17] = (byte) (a ? 0x02 : 0x04);

        /** Magnetometer **/

        data[18] = 0x5C; /** X - Value: 0.86 **/
        data[19] = 0x03;
        data[20] = 0x6A; /** Y - Value: 4.202 **/
        data[21] = 0x10;
        data[22] = 0x00; /** Z - Value: 0 if id = 0, 1.024 otherwise **/
        data[23] = (byte) (a ? 0x00 : 0x04);

        /** Inclination **/

        data[24] = 0x3C; /** X - Value: 6 **/
        data[25] = 0x00;
        data[26] = 0x32; /** Y - Value: 5 **/
        data[27] = 0x00;
        data[28] = 0x00; /** Z - Value: 25.6 if id = 0, 76.8 otherwise **/
        data[29] = (byte) (a ? 0x01 : 0x03);

        /** Compass **/

        data[30] = 0x1A; /** Compass: 1692.2 if id = 0, 6402.6 otherwise **/
        data[31] = (byte) (a ? 0x42 : 0xFA);

        /** Angular Position **/

        data[32] = 0x00; /** W - Value: 1.5 **/
        data[33] = 0x60;
        data[34] = 0x00; /** X - Value: 0.5 **/
        data[35] = 0x20;
        data[36] = 0x00; /** Y - Value: 0.25 **/
        data[37] = 0x10;
        data[38] = 0x00; /** Z - Value: 1 if id = 0, 0.15625 otherwise **/
        data[39] = (byte) (a ? 0x40 : 0x01);

        HidInMsg msg = new FreespaceMsgInMotionEngineOutput(data, DecoderFactory.HID_MESSAGE_VERSION.FSP16x);
        return msg;
    }

    public static class fakeHillcrest extends Device {

        public fakeHillcrest(int id, String name, int vendorId, int productId, int hVer) {
            super(id, name, vendorId, productId, hVer);
        }

        public boolean getFf0() {
            System.out.println("Hey look I was actually called (0)");
            return true;
        }

        public boolean getFf1() {
            System.out.println("Hey look I was actually called (1)");
            return true;
        }
    }
}
