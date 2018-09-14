package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;

/**
 * Created by phanau on 9/14/18.
 */

/**
 * This OpMode illustrates the basics of using the 2018-19 VuforiaLib_RoverRuckus library to determine
 * positioning and orientation of robot on the FTC field.
 */

@Autonomous(name="Test: Vuforia Navigation Test 4", group ="Test")
//@Disabled
public class VuforiaNavigationTest4 extends OpMode {

    VuforiaLib_RoverRuckus mVLib;

    @Override public void init() {
        /**
         * Start up Vuforia using VuforiaLib_FTC2016
         */
        mVLib = new VuforiaLib_RoverRuckus();
        mVLib.init(this, null);     // pass it this OpMode (so it can do telemetry output) and use its license key for now
    }

    @Override public void start()
    {
        /** Start tracking the data sets we care about. */
        mVLib.start();
    }

    @Override public void loop()
    {
        mVLib.loop(true);       // update location info and do debug telemetry
    }

    @Override public void stop()
    {
        mVLib.stop();
    }

}
