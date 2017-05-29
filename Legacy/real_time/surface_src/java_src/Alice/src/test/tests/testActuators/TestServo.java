package tests.testActuators;

import com.roboclub.robobuggy.main.Robot;

/**
 * @author Trevor Decker
 * @author Kevin
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: tester class for the servo
 */

public class TestServo {
    private static boolean running;

    public static void main(String[] args) {
        // TODO Auto-generated method stub
        running = true;
        LeftRightSweep();

    }

    //drives straight then turns right
    public static void straightAndRight() {
        int count = 0;
        int straightOneTime = 10;
        int turnTime = 10;
        int traightTwoTime = 10;
        while (running && count < straightOneTime) {
            try {
                Thread.sleep(100);
                //TODO send angle message
            } catch (Exception e) {
                Thread.currentThread().interrupt();
            }
        }
        //send turn angle
        count = 0;
        while (running && count < turnTime) {
            try {
                Thread.sleep(100);
                //TODO send angle message
            } catch (Exception e) {
                Thread.currentThread().interrupt();
            }
        }
        count = 0;
        while (running && count < straightTwoTime) {
            try {
                Thread.sleep(100);
                //TODO send angle message
            } catch (Exception e) {
                Thread.currentThread().interrupt();
            }
        }

        Thread.sleep(100);

    }


    public static void LeftRightSweep() {
        //currently a test of the servo, todo change this to a seprate test class
        int i = 0;
        boolean up = true;
        while (running) {
            // Robot.WriteAngle(i);
            if (up)
                i += 5;
            else
                i -= 5;
            if (i >= 90)
                up = false;
            else if (i <= 0)
                up = true;

            try {
                Thread.sleep(100);
            } catch (Exception e) {
                Thread.currentThread().interrupt();
            }
        }

        Robot.ShutDown();

    }

}
