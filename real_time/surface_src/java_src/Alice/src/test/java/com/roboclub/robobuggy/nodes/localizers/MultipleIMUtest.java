package com.roboclub.robobuggy.nodes.localizers;

import com.hcrest.jfreespace.Device;
import com.hcrest.jfreespace.FreespaceErrorCodes;
import com.hcrest.jfreespace.inreport.DecoderFactory;
import com.hcrest.jfreespace.inreport.FreespaceMsgInMotionEngineOutput;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.hcrest.jfreespace.outreport.HidOutMsg;
import com.roboclub.robobuggy.messages.*;
import com.roboclub.robobuggy.nodes.sensors.HillCrestImuNode;
import com.roboclub.robobuggy.ros.*;
import org.junit.Test;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Created by Alex on 11/10/2016.
 */
public class MultipleIMUtest {
    private Subscriber accSub = new Subscriber("test", NodeChannel.IMU_ACCELERATION.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUAccelerationMessage){
            processMessage(m);
            System.out.println("Received Acceleration Message");
            /*
            System.out.println("Acceleration Message; Topic is : " + topicName);
            IMUAccelerationMessage mes = ((IMUAccelerationMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getXAccel());
            System.out.println("Y: " + mes.getYAccel());
            System.out.println("Z: " + mes.getZAccel());
            System.out.println("\n====================================================");
            */
        }
    });

    private Subscriber linAccSub = new Subscriber("test", NodeChannel.IMU_LINEAR_ACC.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMULinearAccelerationMessage){
            processMessage(m);
            System.out.println("Received Linear Acceleration Message");
            /*
            System.out.println("Linear Acceleration Message; Topic is : " + topicName);
            IMULinearAccelerationMessage mes = ((IMULinearAccelerationMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getX());
            System.out.println("Y: " + mes.getY());
            System.out.println("Z: " + mes.getZ());
            System.out.println("\n====================================================");
            */
        }
    });

    private Subscriber angVelSub = new Subscriber("test", NodeChannel.IMU_ANG_VEL.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUAngularVelocityMessage){
            processMessage(m);
            System.out.println("Received Angular Velocity Message");
            /*
            System.out.println("Angular Velocity Message; Topic is : " + topicName);
            IMUAngularVelocityMessage mes = ((IMUAngularVelocityMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getX());
            System.out.println("Y: " + mes.getY());
            System.out.println("Z: " + mes.getZ());
            System.out.println("\n====================================================");
            */
        }
    });

    private Subscriber magSub = new Subscriber("test", NodeChannel.IMU_MAGNETIC.getMsgPath(), (topicName, m) -> {
        if(m instanceof MagneticMeasurement){
            processMessage(m);
            System.out.println("Received Magnetic Message");
            /*
            System.out.println("Magnetic Measurement Message; Topic is : " + topicName);
            MagneticMeasurement mes = ((MagneticMeasurement)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getmagX());
            System.out.println("Y: " + mes.getmagY());
            System.out.println("Z: " + mes.getmagZ());
            System.out.println("\n====================================================");
            */
        }
    });

    private Subscriber incSub = new Subscriber("test", NodeChannel.IMU_INCLINATION.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUInclinationMessage){
            processMessage(m);
            System.out.println("Received Inclination Message");
            /*
            System.out.println("Inclination Message; Topic is : " + topicName);
            IMUInclinationMessage mes = ((IMUInclinationMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("X: " + mes.getXInclination());
            System.out.println("Y: " + mes.getYInclination());
            System.out.println("Z: " + mes.getZInclination());
            System.out.println("\n====================================================");
            */
        }
    });

    private Subscriber compSub = new Subscriber("test", NodeChannel.IMU_COMPASS.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUCompassMessage){
            processMessage(m);
            System.out.println("Received Compass Message");
            /*
            System.out.println("Compass Message; Topic is : " + topicName);
            IMUCompassMessage mes = ((IMUCompassMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            System.out.println("Heading: " + mes.getCompassHeading());
            System.out.println("\n====================================================");
            */
        }
    });

    private Subscriber angPosSub = new Subscriber("test", NodeChannel.IMU_ANG_POS.getMsgPath(), (topicName, m) -> {
        if(m instanceof IMUAngularPositionMessage){
            processMessage(m);
            System.out.println("Received Angular Position Message");
            /*
            IMUAngularPositionMessage mes = ((IMUAngularPositionMessage)m);
            System.out.println("IMU: " + mes.getIMU());
            double[][] rot = mes.getRot();
            System.out.println("Values");
            for(int i = 0; i < rot.length; i++) {
                for(int j = 0; j < rot[i].length; j++) {
                    System.out.println("rot["+i+"]["+j+"] = " + rot[i][j]);
                }
            }
            System.out.println("\n====================================================");
            */

        }
    });

    private static ArrayList<Message> messages = new ArrayList<>();

    double EPS = Math.pow(10, -5);
    long waitTime = 3000;

    @Test
    public void testStandardIMUData() {
        System.out.println("Begin Tests");
        System.out.println("\n====================================================\n");
        fakeHillcrest a = new fakeHillcrest(1, "HillCrest A", 1, 1, 1);
        fakeHillcrest b = new fakeHillcrest(2, "HillCrest B", 1, 1, 1);

        HillCrestImuNode node = new HillCrestImuNode();

        node.freespaceDeviceInserted(a);
        node.freespaceDeviceInserted(b);

        HidInMsg msgA = makeMessage(0);
        HidInMsg msgB = makeMessage(1);


        System.out.println("Testing MSG 0 with IMU 0");
        System.out.println("====================================================");
        node.handleRead(a, msgA, 34, System.currentTimeMillis());

        verifyTest(waitTime, 0, 0);

        System.out.println("Success\n");
        System.out.println("Testing MSG 0 with IMU 1");
        System.out.println("====================================================");
        node.handleRead(b, msgA, 34, System.currentTimeMillis());

        verifyTest(waitTime, 0, 1);

        System.out.println("Success\n");
        System.out.println("Testing MSG 1 with IMU 0");
        System.out.println("====================================================");
        node.handleRead(a, msgB, 34, System.currentTimeMillis());

        verifyTest(waitTime, 1, 0);

        System.out.println("Success\n");
        System.out.println("Testing MSG 1 with IMU 1");
        System.out.println("====================================================");
        node.handleRead(b, msgB, 34, System.currentTimeMillis());

        verifyTest(waitTime, 1, 1);

        System.out.println("Success\n");
    }

    private static HidInMsg makeMessage(int id) {

        boolean a = id == 0;

        byte[] data = new byte[1000];

        /** Acceleration **/

        data[0] = 0x20; /** X - Value: 8 **/
        data[1] = 0x03;
        data[2] = 0x50; /** Y - Value: 3.36 **/
        data[3] = 0x01;
        data[4] = 0x00; /** Z - Value: 40.96 for id = 0, 5.12 otherwise **/
        data[5] = (byte) (a ? 0x10 : 0x02);

        /** Linear Acceleration **/

        data[6] = 0x50; /** X - Value: 3.36 **/
        data[7] = 0x01;
        data[8] = 0x20; /** Y - Value: 8 **/
        data[9] = 0x03;
        data[10] = 0x00; /** Z - Value: 2.56 for id = 0, 0 otherwise **/
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

        data[30] = 0x1A; /** Compass: 1692.2 if id = 0, -151.0 otherwise **/
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

        HidInMsg msg = new fakeMsg(data, null);
        return msg;
    }

    public static void processMessage(Message m) {
        messages.add(m);
    }

    public void verifyTest(long delayinMS, int messageID, int imuID) {
        boolean z = messageID == 0;

        System.out.println("Waiting " + delayinMS + " ms ....");
        try {
            TimeUnit.MILLISECONDS.sleep(delayinMS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        System.out.println("Testing Message Results");
        while(!messages.isEmpty()) {
            Message m = messages.remove(0);
            if(m instanceof IMUAccelerationMessage) {
                IMUAccelerationMessage a = (IMUAccelerationMessage) m;
                assert a.getIMU() == imuID;
                assert approxEq(a.getXAccel(), 8, EPS);
                assert approxEq(a.getYAccel(), 3.36, EPS);
                assert approxEq(a.getZAccel(), z ? 40.96 : 5.12, EPS);
            } else if(m instanceof IMULinearAccelerationMessage) {
                IMULinearAccelerationMessage a = (IMULinearAccelerationMessage) m;
                assert a.getIMU() == imuID;
                assert approxEq(a.getX(), 3.36, EPS);
                assert approxEq(a.getY(), 8, EPS);
                assert approxEq(a.getZ(), z ? 2.56 : 0, EPS);
            } else if(m instanceof IMUAngularVelocityMessage) {
                IMUAngularVelocityMessage a = (IMUAngularVelocityMessage) m;
                assert a.getIMU() == imuID;
                assert approxEq(a.getX(), 8.5, EPS);
                assert approxEq(a.getY(), 9.5, EPS);
                assert approxEq(a.getZ(), z ? 63.8 : 115, EPS);
            } else if(m instanceof MagneticMeasurement) {
                MagneticMeasurement a = (MagneticMeasurement) m;
                assert a.getIMU() == imuID;
                assert approxEq(a.getmagX(), 0.86, EPS);
                assert approxEq(a.getmagY(), 4.202, EPS);
                assert approxEq(a.getmagZ(), z ? 0 : 1.024, EPS);
            } else if(m instanceof IMUInclinationMessage) {
                IMUInclinationMessage a = (IMUInclinationMessage) m;
                assert a.getIMU() == imuID;
                assert approxEq(a.getXInclination(), 6, EPS);
                assert approxEq(a.getYInclination(), 5, EPS);
                assert approxEq(a.getZInclination(), z ? 25.6 : 76.8, EPS);
            } else if(m instanceof IMUCompassMessage) {
                IMUCompassMessage a = (IMUCompassMessage) m;
                assert a.getIMU() == imuID;
                assert approxEq(a.getCompassHeading(), z ? 1692.2 : -151.0, EPS);
            } else if(m instanceof IMUAngularPositionMessage) {
                double[][] rot = ((IMUAngularPositionMessage) m).getRot();

                assert rot.length == 3;
                assert rot[0].length == 3;
                assert rot[1].length == 3;
                assert rot[2].length == 3;
                assert rot[0][0] == (z ? -1.125 : 0.87451171875);
                assert rot[0][1] == (z ? -2.75 : 0.203125);
                assert rot[0][2] == (z ? 1.75 : 0.765625);
                assert rot[1][0] == (z ? 3.25 : 0.296875);
                assert rot[1][1] == (z ? -1.5 : 0.49951171875);
                assert rot[1][2] == (z ? -1.0 : -1.4921875);
                assert rot[2][0] == (z ? 0.25 : -0.734375);
                assert rot[2][1] == (z ? 2.0 : 1.5078125);
                assert rot[2][2] == (z ? 0.375 : 0.375);
            } else {
                System.out.println("Class not found");
                System.out.println("Class given was " + m.getClass().toString());
                assert false;
            }
        }
    }

    public boolean approxEq(double x, double y, double epsilon) {
        return y - epsilon <= x && x <= y + epsilon;
    }

    public static class fakeHillcrest extends Device {

        public fakeHillcrest(int id, String name, int vendorId, int productId, int hVer) {
            super(id, name, vendorId, productId, hVer);
        }

        public FreespaceErrorCodes sendMessageAsync(HidOutMsg msg) {
            return null;
        }
    }

    public static class fakeMsg extends FreespaceMsgInMotionEngineOutput {
        private byte[] meData_;

        public fakeMsg(byte[] msg, DecoderFactory.HID_MESSAGE_VERSION version) {
            super(msg, DecoderFactory.HID_MESSAGE_VERSION.FSP2xx);
            this.meData_ = msg;
        }

        public int[] getMeData() {
            int[] ret = new int[meData_.length];
            for(int i = 0; i < meData_.length; i++) {
                ret[i] = meData_[i];
            }
            return ret;
        }

        public int getFormatSelect() {
            return 1;
        }

        public boolean getFf0() {
            return true;
        }

        public boolean getFf1() {
            return true;
        }

        public boolean getFf2() {
            return true;
        }

        public boolean getFf3() {
            return true;
        }

        public boolean getFf4() {
            return true;
        }

        public boolean getFf5() {
            return true;
        }

        public boolean getFf6() {
            return true;
        }
    }
}
