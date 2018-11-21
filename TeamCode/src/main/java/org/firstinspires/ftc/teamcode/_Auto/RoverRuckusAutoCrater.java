package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

@Autonomous(name = "CraterAutoMain")
public class RoverRuckusAutoCrater extends OpMode{
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    private AutoLib.Sequence mSeq = new AutoLib.LinearSequence();

    public void init() {
        robot.init(hardwareMap);
        robot.lift.setPower(-.5);
        robot.lift2.setPower(-.5);

        mSeq.add(new AutoLib.MoveByEncoderStep(robot.lift, robot.lift2, 1f, 500, true));
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 4250, true));
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(mSeq.loop()) requestOpModeStop();
    }
}
