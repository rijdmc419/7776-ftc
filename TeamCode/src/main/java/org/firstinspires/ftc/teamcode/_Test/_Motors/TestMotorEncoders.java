package org.firstinspires.ftc.teamcode._Test._Motors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;

/**
 * Created by phanau on 2/23/18.

 * Test hardware gyro
 */
@Autonomous(name="Test: Motor Encoder Test 1", group ="Test")
//@Disabled
public class TestMotorEncoders extends OpMode {

    DcMotor mMotor;
    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done

    public TestMotorEncoders() {
    }

    public void init() {
        // get hardware
        AutoLib.HardwareFactory mf = null;
        final boolean debug = false;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        mMotor = mf.getDcMotor("fr");

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a Step sequence that rotates a motor one rev forward and then one rev backward
        double power = 0.8;
        int count = 28*60;    // shaft encoder at 28 ppr * 60:1 gearbox
        boolean stop = true;
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor, power, count, stop));
        mSequence.add(new AutoLib.LogTimeStep(this, "wait", 1.0));
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor, power, -count, stop));

    }

    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    public void stop() {
    }

}
