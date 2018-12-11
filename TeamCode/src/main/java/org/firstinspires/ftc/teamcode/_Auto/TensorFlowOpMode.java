package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

@Autonomous(name = "TensorFlowOpmode")
public class TensorFlowOpMode extends OpMode{
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    private AutoLib.Sequence mSeq = new AutoLib.LinearSequence();
    private AutoLib.TurnByEncoderStep turnStep;
    private AutoLib.TurnByEncoderStep returnStep;
    private AutoLib.TurnByEncoderStep turnToCraterStep;
    private AutoLib.TurnByEncoderStep driveAfterTurn;

    public void init() {
        robot.init(hardwareMap);
        turnStep = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 0, 0, 0, 0, true);
        mSeq.add(new TensorFlowStepNew(this, turnStep, robot, returnStep, turnToCraterStep, driveAfterTurn));
        /*mSeq.add(turnStep);
        mSeq.add(returnStep);*/
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(mSeq.loop()) requestOpModeStop();
    }
}
