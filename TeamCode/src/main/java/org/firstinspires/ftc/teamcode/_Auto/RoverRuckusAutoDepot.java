package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

@Autonomous(name = "DepotAutoMain")
public class RoverRuckusAutoDepot extends OpMode{
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    private AutoLib.Sequence mSeq = new AutoLib.LinearSequence();
    private AutoLib.TurnByEncoderStep turnStep;
    private AutoLib.TurnByEncoderStep returnStep;
    private AutoLib.TurnByEncoderStep turnToCraterStep;
    private AutoLib.TurnByEncoderStep driveAfterTurn;

    public void init() {
        robot.init(hardwareMap);
        robot.lift.setPower(-.5);
        robot.lift2.setPower(-.5);
        turnStep = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 1.0f, 0, 0, true);
        returnStep = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 1.0f, 0, 0, true);
        turnToCraterStep = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 1.0f, 0, 0, true);
        driveAfterTurn = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 1.0f, 0, 0, true);

        mSeq.add(new TensorFlowStep(this, turnStep, robot, returnStep, turnToCraterStep, driveAfterTurn));
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.lift, robot.lift2, 1f, 500, true));
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 500, true));
        mSeq.add(turnStep);
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 3000, true));
        mSeq.add(returnStep);
        mSeq.add(driveAfterTurn);
        mSeq.add(turnToCraterStep);
        mSeq.add(new AutoLib.ServoStep(robot.markerServo, -1));
        mSeq.add(new AutoLib.LogTimeStep(this, "wait for servo", 1));
        mSeq.add(new AutoLib.ServoStep(robot.markerServo, 1));
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(mSeq.loop()) requestOpModeStop();
    }
}