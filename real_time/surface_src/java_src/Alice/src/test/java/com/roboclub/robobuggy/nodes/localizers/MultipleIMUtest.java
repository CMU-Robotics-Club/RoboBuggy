package com.roboclub.robobuggy.nodes.localizers;

import com.hcrest.jfreespace.Device;
import com.hcrest.jfreespace.inreport.DecoderFactory;
import com.hcrest.jfreespace.inreport.FreespaceMsgInMotionEngineOutput;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.hcrest.jfreespace.inreport.HidInMsgType;
import com.roboclub.robobuggy.messages.*;
import com.roboclub.robobuggy.nodes.sensors.HillCrestImuNode;
import com.roboclub.robobuggy.ros.*;

/**
 * Created by Alex on 11/10/2016.
 */
public class MultipleIMUtest {
    private Subscriber linAccSub = new Subscriber("test", NodeChannel.IMU_LINEAR_ACC.getMsgPath(), new MessageListener() {
        @Override
        public void actionPerformed(String topicName, Message m) {
            if(m instanceof IMULinearAccelerationMessage){
                System.out.println("Linear Acceleration Message; Topic is : " + topicName);
                IMULinearAccelerationMessage mes = ((IMULinearAccelerationMessage)m);
                System.out.println("IMU: " + mes.getIMU());
                System.out.println("X: " + mes.getX());
                System.out.println("Y: " + mes.getY());
                System.out.println("Z: " + mes.getZ());
                System.out.println("\n====================================================");
            }
        }
    });

    private Subscriber linAccNoGravSub = new Subscriber("test", NodeChannel.IMU_LINEAR_NO_GRAV.getMsgPath(), new MessageListener() {
        @Override
        public void actionPerformed(String topicName, Message m) {
            if(m instanceof IMULinearAccelerationMessage){
                System.out.println("Linear Acceleration Message; Topic is : " + topicName);
                IMULinearAccelerationMessage mes = ((IMULinearAccelerationMessage)m);
                System.out.println("IMU: " + mes.getIMU());
                System.out.println("X: " + mes.getX());
                System.out.println("Y: " + mes.getY());
                System.out.println("Z: " + mes.getZ());
                System.out.println("\n====================================================");
            }
        }
    });

    private Subscriber angVelSub = new Subscriber("test", NodeChannel.IMU_ANG_VEL.getMsgPath(), new MessageListener() {
        @Override
        public void actionPerformed(String topicName, Message m) {
            if(m instanceof IMUAngularVelocityMessage){
                System.out.println("Linear Acceleration Message; Topic is : " + topicName);
                IMUAngularVelocityMessage mes = ((IMUAngularVelocityMessage)m);
                System.out.println("IMU: " + mes.getIMU());
                System.out.println("X: " + mes.getX());
                System.out.println("Y: " + mes.getY());
                System.out.println("Z: " + mes.getZ());
                System.out.println("\n====================================================");
            }
        }
    });

    private Subscriber magSub = new Subscriber("test", NodeChannel.IMU_MAGNETIC.getMsgPath(), new MessageListener() {
        @Override
        public void actionPerformed(String topicName, Message m) {
            if(m instanceof MagneticMeasurement){
                System.out.println("Linear Acceleration Message; Topic is : " + topicName);
                MagneticMeasurement mes = ((MagneticMeasurement)m);
                System.out.println("IMU: " + mes.getIMU());
                System.out.println("X: " + mes.getmagX());
                System.out.println("Y: " + mes.getmagY());
                System.out.println("Z: " + mes.getmagZ());
                System.out.println("\n====================================================");
            }
        }
    });

    private Subscriber tempSub = new Subscriber("test", NodeChannel.IMU_TEMP.getMsgPath(), new MessageListener() {
        @Override
        public void actionPerformed(String topicName, Message m) {
            if(m instanceof IMUTemperatureMessage){
                System.out.println("Linear Acceleration Message; Topic is : " + topicName);
                IMUTemperatureMessage mes = ((IMUTemperatureMessage)m);
                System.out.println("IMU: " + mes.getIMU());
                System.out.println("Temperature: " + mes.getTemp());
                System.out.println("\n====================================================");
            }
        }
    });

    private Subscriber angPosSub = new Subscriber("test", NodeChannel.IMU_ANG_POS.getMsgPath(), new MessageListener() {
        @Override
        public void actionPerformed(String topicName, Message m) {
            if(m instanceof IMUAngularPositionMessage){
                System.out.println("Linear Acceleration Message; Topic is : " + topicName);
                IMUAngularPositionMessage mes = ((IMUAngularPositionMessage)m);
                System.out.println("IMU: " + mes.getIMU());
                double[][] rot = mes.getRot();
                System.out.println("Values");
                System.out.println(rot.toString());
                System.out.println("\n====================================================");
            }
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

    public static HidInMsg makeMessage(int id) {
        byte[] data = new byte[34];
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        data[5] = 0;

        boolean a = id == 0;

        /** Linear Acceleration **/

        data[6] = 0x00; /** X - Value: 8 **/
        data[7] = 0x20;
        data[8] = 0x00; /** Y - Value: 4 **/
        data[9] = 0x10;
        data[10] = 0x00; /** Z - Value: 4 for id = 0, 8 otherwise **/
        data[11] = (byte) (a ? 0x10 : 0x20);

        /** Linear Acceleration (No Gravity) **/

        data[12] = 0x00; /** X - Value: 8.5 **/
        data[13] = 0x22;
        data[14] = 0x00; /** Y - Value: 9.5 **/
        data[15] = 0x26;
        data[16] = 0x00; /** Z - Value: 1 if id = 0, 4 otherwise **/
        data[17] = (byte) (a ? 0x04 : 0x10);

        /** Angular Velocity **/

        data[18] = 0x00; /** X - Value: 1.5 **/
        data[19] = 0x08;
        data[20] = 0x00; /** Y - Value: 4.25 **/
        data[21] = 0x11;
        data[22] = 0x00; /** Z - Value: 0 if id = 0, 1 otherwise **/
        data[23] = (byte) (a ? 0x00 : 0x04);

        /** Magnetometer **/

        data[24] = 0x00; /** X - Value: -6 **/
        data[25] = (byte) 0xA0;
        data[26] = 0x00; /** Y - Value: -5 **/
        data[27] = (byte) 0xB0;
        data[28] = 0x00; /** Z - Value: 1 if id = 0, -1 otherwise **/
        data[29] = (byte) (a ? 0x10 : 0xF0);

        /** Temperature **/

        data[30] = 0x00; /** Temperature: 128 if id = 0, 4 otherwise **/
        data[31] = (byte) (a ? 0x40 : 0x01);

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
            return true;
        }

        public boolean getFf1() {
            return true;
        }
    }
}
