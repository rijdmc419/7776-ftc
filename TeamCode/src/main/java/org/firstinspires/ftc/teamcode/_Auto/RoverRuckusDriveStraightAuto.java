package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

@Autonomous(name = "Drive Straight Auto")
public class RoverRuckusDriveStraightAuto extends OpMode{
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    private AutoLib.Sequence mSeq;
    private boolean bDone;

    public void init() {
         bDone = false;
         mSeq = new AutoLib.LinearSequence();

        robot.init(hardwareMap);
        //mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 0.5f, 450, true));
      //  mSeq.add(new AutoLib.AzimuthCountedDriveStep(this,));
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSeq.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");

    }
}
