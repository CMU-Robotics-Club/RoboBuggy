package com.roboclub.robobuggy.nodes.localizers;

import com.hcrest.jfreespace.Device;
import com.hcrest.jfreespace.DeviceListenerInterface;
import com.hcrest.jfreespace.FreespaceErrorCodes;
import com.hcrest.jfreespace.inreport.DecoderFactory;
import com.hcrest.jfreespace.inreport.FreespaceMsgInMotionEngineOutput;
import com.hcrest.jfreespace.inreport.HidInMsg;
import com.hcrest.jfreespace.outreport.HidOutMsg;
import com.roboclub.robobuggy.messages.*;
import com.roboclub.robobuggy.nodes.sensors.HillCrestImuNode;
import com.roboclub.robobuggy.ros.*;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Created by Alex on 11/10/2016.
 */
public class MultipleIMUtest {
    private static ArrayList<ArrayList<Message>> messages;

    private final double EPS = Math.pow(10, -5);
    private final long waitTime = 15 * 1000;
    private final int numExpectedMessages = 7;

    private static HillCrestImuNode node;

    private static FakeHillcrest[] hillcrests;

    private static boolean setUp = false;

    @Test
    public void testIndividualIMU() {
        if(!setUp) {
            oneTimeSetup();
        } else {
            node.freespaceDeviceInserted(hillcrests[0]);
            node.freespaceDeviceInserted(hillcrests[1]);
        }

        HidInMsg msg0 = makeMessage(0);
        HidInMsg msg1 = makeMessage(1);

        int[] zeroRay = {0};
        int[] oneRay = {1};

        System.out.println("Testing MSG 0 with IMU 0");
        System.out.println("====================================================");

        for(int i = 0; i < messages.size(); i++) {
            assert messages.get(i).size() == 0;
        }
        node.handleRead(hillcrests[0], msg0, 34, System.currentTimeMillis());
        testMessages(waitTime, zeroRay, 0);

        System.out.println("Success\n");

        System.out.println("Testing MSG 0 with IMU 1");
        System.out.println("====================================================");

        for(int i = 0; i < messages.size(); i++) {
            assert messages.get(i).size() == 0;
        }

        node.handleRead(hillcrests[1], msg0, 34, System.currentTimeMillis());
        testMessages(waitTime, zeroRay, 1);

        System.out.println("Success\n");

        System.out.println("Testing MSG 1 with IMU 0");
        System.out.println("====================================================");

        for(int i = 0; i < messages.size(); i++) {
            assert messages.get(i).size() == 0;
        }

        node.handleRead(hillcrests[0], msg1, 34, System.currentTimeMillis());
        testMessages(waitTime, oneRay, 0);

        System.out.println("Success\n");

        System.out.println("Testing MSG 1 with IMU 1");
        System.out.println("====================================================");

        for(int i = 0; i < messages.size(); i++) {
            assert messages.get(i).size() == 0;
        }

        node.handleRead(hillcrests[1], msg1, 34, System.currentTimeMillis());
        testMessages(waitTime, oneRay, 1);

        System.out.println("Success\n");

        node.freespaceDeviceRemoved(hillcrests[0]);
        node.freespaceDeviceRemoved(hillcrests[1]);
    }

    @Test
    public void testMultipleImus() {
        if(!setUp) {
            oneTimeSetup();
        } else {
            node.freespaceDeviceInserted(hillcrests[0]);
            node.freespaceDeviceInserted(hillcrests[1]);
        }

        HidInMsg msg0 = makeMessage(0);
        HidInMsg msg1 = makeMessage(1);

        int[] zeroRay = {0};
        int[] oneRay = {1};

        System.out.println("Testing MSG 0 with both IMUs");
        System.out.println("====================================================");

        node.handleRead(hillcrests[0], msg0, 34, System.currentTimeMillis());
        node.handleRead(hillcrests[1], msg0, 34, System.currentTimeMillis());
        testMessages(waitTime, zeroRay, 0);
        testMessages(0, zeroRay, 1);          //No need for a wait time because the messages are already there

        System.out.println("Success\n");

        System.out.println("Testing MSG 1 with both IMUs");
        System.out.println("====================================================");

        node.handleRead(hillcrests[0], msg1, 34, System.currentTimeMillis());
        node.handleRead(hillcrests[1], msg1, 34, System.currentTimeMillis());
        testMessages(waitTime, oneRay, 0);
        testMessages(0, oneRay, 1);          //No need for a wait time because the messages are already there

        System.out.println("Success\n");

        node.freespaceDeviceRemoved(hillcrests[0]);
        node.freespaceDeviceRemoved(hillcrests[1]);
    }

    @Test
    public void testMultipleMessages() {
        if(!setUp) {
            oneTimeSetup();
        } else {
            node.freespaceDeviceInserted(hillcrests[0]);
            node.freespaceDeviceInserted(hillcrests[1]);
        }


        node.freespaceDeviceRemoved(hillcrests[0]);
        node.freespaceDeviceRemoved(hillcrests[1]);
    }

    public void testMessages (long delayinMS, int[] messageID, int imuID) {
        boolean[] z = new boolean[messageID.length];

        for(int i = 0; i < messageID.length; i++) {
            z[i] = messageID[i] == 0;
        }

        System.out.println("Waiting " + delayinMS + " ms ....");
        try {
            TimeUnit.MILLISECONDS.sleep(delayinMS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(imuID >= messages.size()) {
            System.out.println("IMU " + imuID + " was not added to the device list");
            assert(false);
        }

        System.out.println("Testing Message Results");

        ArrayList<Message> imuMessage = messages.get(imuID);

        System.out.println("Size of Queue is: " + imuMessage.size());
        //assert(imuMessage.size() == numExpectedMessages * messageID.length);
        int i = 0;
        while(!imuMessage.isEmpty()) {
            System.out.println("Testing Message #" + i + " in queue");
            i++;
            Message m = imuMessage.remove(0);
            if(m instanceof IMUAccelerationMessage) {
                IMUAccelerationMessage a = (IMUAccelerationMessage) m;
                accelerationTest(a, z, imuID);
            } else if(m instanceof IMULinearAccelerationMessage) {
                IMULinearAccelerationMessage a = (IMULinearAccelerationMessage) m;
                linAccelerationTest(a, z, imuID);
            } else if(m instanceof IMUAngularVelocityMessage) {
                IMUAngularVelocityMessage a = (IMUAngularVelocityMessage) m;
                angularVelTest(a, z, imuID);
            } else if(m instanceof MagneticMeasurement) {
                MagneticMeasurement a = (MagneticMeasurement) m;
                magneticTest(a, z, imuID);
            } else if(m instanceof IMUInclinationMessage) {
                IMUInclinationMessage a = (IMUInclinationMessage) m;
                inclinationTest(a, z, imuID);
            } else if(m instanceof IMUCompassMessage) {
                IMUCompassMessage a = (IMUCompassMessage) m;
                compassTest(a, z, imuID);
            } else if(m instanceof IMUAngularPositionMessage) {
                IMUAngularPositionMessage a = (IMUAngularPositionMessage) m;
                angularPositionTest(a, z, imuID);
            } else {
                System.out.println("Class not found");
                if(m == null) {
                    System.out.println("Message was null");
                    //assert false;
                } else {
                    System.out.println("Class given was " + m.getClass().toString());
                    assert false;
                }
            }
        }

        assert imuMessage.size() == 0;
    }

    public void accelerationTest(IMUAccelerationMessage a, boolean[] z, int imuID) {
        assert a.getImuID() == imuID;
        assert approxEq(a.getXAccel(), 8, EPS);
        assert approxEq(a.getYAccel(), 3.36, EPS);
        for(boolean b : z) {
            if (approxEq(a.getZAccel(), b ? 40.96 : 5.12, EPS)) {
                return;
            }
        }
        assert false;
    }

    public void linAccelerationTest(IMULinearAccelerationMessage a, boolean[] z, int imuID) {
        assert a.getImuID() == imuID;
        assert approxEq(a.getX(), 3.36, EPS);
        assert approxEq(a.getY(), 8, EPS);
        for(boolean b : z) {
            if (approxEq(a.getZ(), b ? 2.56 : 0, EPS)) {
                return;
            }
        }
        assert false;
    }

    public void angularVelTest(IMUAngularVelocityMessage a, boolean[] z, int imuID) {
        assert a.getImuID() == imuID;
        assert approxEq(a.getX(), 8.5, EPS);
        assert approxEq(a.getY(), 9.5, EPS);
        for(boolean b : z) {
            if (approxEq(a.getZ(), b ? 63.8 : 115, EPS)) {
                return;
            }
        }
        assert false;
    }

    public void magneticTest(MagneticMeasurement a, boolean[] z, int imuID) {
        assert a.getImuID() == imuID;
        assert approxEq(a.getmagX(), 0.86, EPS);
        assert approxEq(a.getmagY(), 4.202, EPS);
        for(boolean b : z) {
            if (approxEq(a.getmagZ(), b ? 0 : 1.024, EPS)) {
                return;
            }
        }
        assert false;
    }

    public void inclinationTest(IMUInclinationMessage a, boolean[] z, int imuID) {
        assert a.getImuID() == imuID;
        assert approxEq(a.getXInclination(), 6, EPS);
        assert approxEq(a.getYInclination(), 5, EPS);
        for(boolean b : z) {
            if (approxEq(a.getZInclination(), b ? 25.6 : 76.8, EPS)) {
                return;
            }
        }
        assert false;
    }

    public void compassTest(IMUCompassMessage a, boolean[] z, int imuID) {
        assert a.getImuID() == imuID;
        for(boolean b : z) {
            if (approxEq(a.getCompassHeading(), b ? 1692.2 : -151.0, EPS)) {
                return;
            }
        }
        assert false;
    }

    public void angularPositionTest(IMUAngularPositionMessage msg, boolean[] z, int imuID) {
        assert msg.getImuID() == imuID;

        double[][] rot = msg.getRot();
        assert rot.length == 3;
        assert rot[0].length == 3;
        assert rot[1].length == 3;
        assert rot[2].length == 3;

        for(boolean b : z) {
            if (approxEq(rot[0][0], (b ? -1.125 : 0.87451171875), EPS) &&
                    approxEq(rot[0][1], (b ? -2.75 : 0.203125), EPS) &&
                    approxEq(rot[0][2], (b ? 1.75 : 0.765625), EPS) &&
                    approxEq(rot[1][0], (b ? 3.25 : 0.296875), EPS) &&
                    approxEq(rot[1][1], (b ? -1.5 : 0.49951171875), EPS) &&
                    approxEq(rot[1][2], (b ? -1.0 : -1.4921875), EPS) &&
                    approxEq(rot[2][0], (b ? 0.25 : -0.734375), EPS) &&
                    approxEq(rot[2][1], (b ? 2.0 : 1.5078125), EPS) &&
                    approxEq(rot[2][2], 0.375, EPS)) {
                return;
            }
        }
        assert false;
    }

    public static void oneTimeSetup() {
        setUp = true;

        messages = new ArrayList<>(2);
        messages.add(new ArrayList<>());
        messages.add(new ArrayList<>());

        FakeHillcrest a = new FakeHillcrest(1, "HillCrest A", 1, 1, 1);
        FakeHillcrest b = new FakeHillcrest(2, "HillCrest B", 1, 1, 1);

        node = new HillCrestImuNode();

        node.freespaceDeviceInserted(a);
        node.freespaceDeviceInserted(b);

        hillcrests = new FakeHillcrest[2];
        hillcrests[0] = a;
        hillcrests[1] = b;

        Subscriber angPosSub = new Subscriber("test", NodeChannel.IMU_ANG_POS.getMsgPath(), (topicName, m) -> {
            int imuID = ((IMUAngularPositionMessage)m).getImuID();
            processMessage(m, imuID);
            System.out.println("Received Angular Position Message");
        });
        Subscriber compSub = new Subscriber("test", NodeChannel.IMU_COMPASS.getMsgPath(), (topicName, m) -> {
            int imuID = ((IMUCompassMessage)m).getImuID();
            processMessage(m, imuID);
            System.out.println("Received Compass Message");
        });
        Subscriber incSub = new Subscriber("test", NodeChannel.IMU_INCLINATION.getMsgPath(), (topicName, m) -> {
            int imuID = ((IMUInclinationMessage)m).getImuID();
            processMessage(m, imuID);
            System.out.println("Received Inclination Message");
        });
        Subscriber magSub = new Subscriber("test", NodeChannel.IMU_MAGNETIC.getMsgPath(), (topicName, m) -> {
            int imuID = ((MagneticMeasurement)m).getImuID();
            processMessage(m, imuID);
            System.out.println("Received Magnetic Message");
        });
        Subscriber angVelSub = new Subscriber("test", NodeChannel.IMU_ANG_VEL.getMsgPath(), (topicName, m) -> {
            int imuID = ((IMUAngularVelocityMessage)m).getImuID();
            processMessage(m, imuID);
            System.out.println("Received Angular Velocity Message");
        });
        Subscriber linAccSub = new Subscriber("test", NodeChannel.IMU_LINEAR_ACC.getMsgPath(), (topicName, m) -> {
            int imuID = ((IMULinearAccelerationMessage)m).getImuID();
            processMessage(m, imuID);
            System.out.println("Received Linear Acceleration Message");
        });
        Subscriber accSub = new Subscriber("test", NodeChannel.IMU_ACCELERATION.getMsgPath(), (topicName, m) -> {
            int imuID = ((IMUAccelerationMessage)m).getImuID();
            processMessage(m, imuID);
            System.out.println("Received Acceleration Message");
        });

        System.out.print("Created Subscribers of types: " );
        System.out.print(accSub.getClass().toString() + ", ");
        System.out.print(linAccSub.getClass().toString() + ", ");
        System.out.print(angVelSub.getClass().toString() + ", ");
        System.out.print(magSub.getClass().toString() + ", ");
        System.out.print(incSub.getClass().toString() + ", ");
        System.out.print(compSub.getClass().toString() + ", ");
        System.out.print(angPosSub.getClass().toString() + "\n");

        System.out.println("Waiting " + 5000 + " ms ....");
        try {
            TimeUnit.MILLISECONDS.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private HidInMsg makeMessage(int id) {

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

        HidInMsg msg = new FakeMsg(data);
        return msg;
    }

    public static void processMessage(Message m, int IMU) {
        if(IMU >= messages.size()) {
            System.out.println("IMU " + IMU + " was not added to the device list");
            assert(false);
        }
        if(m == null) {
            System.out.println("Message is NULL");
            assert false;
        }
        messages.get(IMU).add(m);
        if(messages.get(IMU).get(0) == null) {
            System.out.println("0th Element is null with size " + messages.get(IMU).size());
        }
    }


    public boolean approxEq(double x, double y, double epsilon) {
        return y - epsilon <= x && x <= y + epsilon;
    }

    public static class FakeHillcrest extends Device {

        public FakeHillcrest(int id, String name, int vendorId, int productId, int hVer) {
            super(id, name, vendorId, productId, hVer);
        }

        @Override
        public FreespaceErrorCodes sendMessageAsync(HidOutMsg msg) {
            return null;
        }

        @Override
        public FreespaceErrorCodes open(DeviceListenerInterface listener) {
            return FreespaceErrorCodes.FREESPACE_SUCCESS;
        }
    }

    public static class FakeMsg extends FreespaceMsgInMotionEngineOutput {
        private int[] store;

        public FakeMsg(byte[] msg) {
            super(msg, DecoderFactory.HID_MESSAGE_VERSION.FSP2xx);

            int[] ret = new int[msg.length];
            for(int i = 0; i < msg.length; i++) {
                ret[i] = msg[i];
            }

            store = ret;
        }

        @Override
        public int[] getMeData() {
            int[] ret = new int[store.length];
            for(int i = 0; i < store.length; i++) {
                ret[i] = store[i];
            }
            return ret;
        }

        @Override
        public int getFormatSelect() {
            return 1;
        }

        @Override
        public boolean getFf0() {
            return true;
        }

        @Override
        public boolean getFf1() {
            return true;
        }

        @Override
        public boolean getFf2() {
            return true;
        }

        @Override
        public boolean getFf3() {
            return true;
        }

        @Override
        public boolean getFf4() {
            return true;
        }

        @Override
        public boolean getFf5() {
            return true;
        }

        @Override
        public boolean getFf6() {
            return true;
        }
    }
}
