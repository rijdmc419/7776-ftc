package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

@Autonomous(name = "AutoTest")
public class RoverRuckusAuto extends OpMode{
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    private AutoLib.Sequence mSeq = new AutoLib.LinearSequence();

    public void init() {
        robot.init(hardwareMap);
        robot.lift.setPower(-1);
        robot.lift2.setPower(-1);
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.lift, robot.lift2, 1f, 575, true));
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 6750, true));
        //mSeq.add(new AutoLib.TurnByEncoderStep(robot.fr, robot.fl, 1.0f, -1.0f, 2000, -2000, true));
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(mSeq.loop()) requestOpModeStop();
    }
}

