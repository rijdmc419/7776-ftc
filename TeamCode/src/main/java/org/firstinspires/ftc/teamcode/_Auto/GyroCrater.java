package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

@Autonomous(name = "CraterTest")
public class GyroCrater extends OpMode{
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    private AutoLib.Sequence mSeq = new AutoLib.LinearSequence();
    private AutoLib.AzimuthTolerancedTurnStep turnStep;
    private AutoLib.AzimuthTolerancedTurnStep returnStep;
    private AutoLib.AzimuthTolerancedTurnStep turnToCraterStep;
    private AutoLib.TurnByEncoderStep driveAfterTurn;
    private int gyroAngle = 0;

    DcMotor mMotors[];
    boolean bDone;                          // motors, some of which can be null: assumed order is fr, br, fl, bl
    BNO055IMUHeadingSensor mGyro;           // gyro to use for heading information
    boolean bSetup;                         // true when we're in "setup mode" where joysticks tweak parameters
    SensorLib.PID mPid;

    public void init() {
        robot.init(hardwareMap);

        bSetup = false;      // start out in Kp/Ki setup mode
        AutoLib.HardwareFactory mf = null;
        final boolean debug = false;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        mMotors = new DcMotor[4];

        mMotors[0] = robot.fr;
        mMotors[1] = robot.br;
        mMotors[2] = robot.fl;
        mMotors[3] = robot.bl;

        mPid = new SensorLib.PID(.025f, 0f, 0f, 1.0f);

        // get hardware IMU and wrap gyro in HeadingSensor object usable below
        mGyro = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));

        robot.joint.setPower(.75f);
        robot.joint2.setPower(.75f);
        robot.joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.joint2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.joint.setTargetPosition(200);
        robot.joint2.setTargetPosition(200);

        turnStep = new AutoLib.AzimuthTolerancedTurnStep(this, 0f, mGyro, mPid, mMotors, .5f, 5, 3);
        returnStep = new AutoLib.AzimuthTolerancedTurnStep(this, 0f, mGyro, mPid, mMotors, .5f, 5, 3);
        turnToCraterStep = new AutoLib.AzimuthTolerancedTurnStep(this, 0f, mGyro, mPid, mMotors, .5f, 5, 3);
        driveAfterTurn = new AutoLib.TurnByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, 1.0f, 1.0f, 0, 0, true);

        mSeq.add(new GyroSampling(this, turnStep, robot, returnStep, turnToCraterStep, driveAfterTurn, gyroAngle));
        mSeq.add(new AutoLib.MoveByEncoderStepTimed(robot.joint, robot.joint2, -.75f, -1450, true));
        mSeq.add(new AutoLib.MoveByTimeStep(robot.extend, robot.extend2, -.75f, 3, true));
        mSeq.add(new AutoLib.GyroInit(mGyro));
        //mSeq.add(new AutoLib.AzimuthCountedDriveStep(this, 0, mGyro, mPid, mMotors, -.4f, -1000, false));
        mSeq.add(turnStep);
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, -.3f, -1200, true));
        /*mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, .3f, 1000, true));
        mSeq.add(new AutoLib.AzimuthTolerancedTurnStep(this, -110f, mGyro, mPid, mMotors, .3f, 5, 3));
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, .3f, 2300, true));
        mSeq.add(new AutoLib.AzimuthTolerancedTurnStep(this, 130f, mGyro, mPid, mMotors, .4f, 5, 3));
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, -.3f, -4200, true));
        //mSeq.add(turnToCraterStep);
        mSeq.add(new AutoLib.AzimuthTolerancedTurnStep(this, 140f, mGyro, mPid, mMotors, -.3f, 5, 3));
        mSeq.add(new AutoLib.MoveByEncoderStep(robot.fr, robot.br, robot.fl, robot.bl, .1f, 6000, true));*/
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(mSeq.loop()) requestOpModeStop();
    }
}
