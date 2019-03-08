package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

@Autonomous(name = "CraterAutoMain")
public class RoverRuckusAutoCrater extends OpMode{
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    private AutoLib.Sequence mSeq = new AutoLib.LinearSequence();
    private AutoLib.TurnByEncoderStep turnStep;
    private AutoLib.TurnByEncoderStep returnStep;
    private AutoLib.TurnByEncoderStep turnToCraterStep;
    private AutoLib.TurnByEncoderStep driveAfterTurn;

    public void init() {
        robot.init(hardwareMap);
      //  robot.lift.setPower(-.5);
     //   robot.lift2.setPower(-.5);
        turnStep = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 1.0f, 0, 0, true);
        returnStep = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 1.0f, 0, 0, true);
        turnToCraterStep = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 1.0f, 0, 0, true);
        driveAfterTurn = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 1.0f, 0, 0, true);

        mSeq.add(new TensorFlowStepNew(this, turnStep, robot, returnStep, turnToCraterStep, driveAfterTurn));
       // mSeq.add(new AutoLib.MoveByEncoderStep(robot.lift, robot.lift2, 1f, 600, true));
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 500, true));
        mSeq.add(turnStep);
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 4000, true));

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(mSeq.loop()) requestOpModeStop();
    }
}
