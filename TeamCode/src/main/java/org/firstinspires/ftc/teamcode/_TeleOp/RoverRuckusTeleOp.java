package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

import java.text.DecimalFormat;

/**
 * Created by bremm on 10/26/18.
 */

@TeleOp(name="Rover Ruckus TeleOp")



public class RoverRuckusTeleOp extends OpMode{
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    double left, right, leftTwo, rightTwo;
    double speedFactor;
    double jointSpeed;
    double jointSpeedFactor = 0.35;
    double extendSpeed;
    double extendSpeedFactor = 0.1;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public  void start() {
    }

    @Override
    public void loop() {
        driveTrainSpeed();

        left = Math.pow(gamepad1.left_stick_y, 3) * speedFactor;
        right = Math.pow(gamepad1.right_stick_y, 3) * speedFactor;

        jointSpeed = 1;
        //intakeSpeed = (-Math.pow(gamepad2.right_stick_y, 3) / 2) + .5f;

        if(gamepad1.a) {
            robot.flapServo.setPosition(1);
        }

        else {
            robot.flapServo.setPosition(.5f);
        }

        extend();
        powSet();
        telemetry();
    }

    void extend(){
        leftTwo = -Math.pow(gamepad2.left_stick_y, 3);
        rightTwo = -Math.pow(gamepad2.right_stick_y, 3);

        if(gamepad2.left_stick_y > 0) {
            leftTwo = -1;
            rightTwo = -1;
        }
        else if(gamepad2.left_stick_y < 0) {
            leftTwo = 1;
            rightTwo = 1;
        }
    }

    void telemetry(){
        DecimalFormat printFormat = new DecimalFormat ("#.###");

        //telemetry.addData("Gamepad 1", gamepad1);
        //telemetry.addData("Gamepad 2", gamepad2);

        telemetry.addData("Encoder", printFormat.format((robot.extend.getCurrentPosition())));
        telemetry.addData("Encoder", printFormat.format((robot.extend2.getCurrentPosition())));
        telemetry.addData("Extend", printFormat.format(extendSpeed));
        telemetry.addData("Joint", printFormat.format(jointSpeed));
        telemetry.addData("Speed Factor", printFormat.format(speedFactor));
        telemetry.addData("Left", printFormat.format(left));
        telemetry.addData("Right", printFormat.format(right));
    }

    void powSet() {
        robot.fl.setPower(-left);
        robot.bl.setPower(-left);
        robot.fr.setPower(-right);
        robot.br.setPower(-right);
        robot.extend2.setPower(leftTwo);
        robot.extend.setPower(rightTwo);

        if(gamepad2.a) {
            robot.intakeServo.setPower(1);
            robot.intakeServo2.setPower(-1);
        }
        else if (gamepad2.b) {
            robot.intakeServo.setPower(-1);
            robot.intakeServo2.setPower(1);
        }
        else {
            robot.intakeServo.setPower(0);
            robot.intakeServo2.setPower(0);
        }

        if(gamepad2.dpad_up) {
            robot.joint.setPower(-.75f);
            robot.joint2.setPower(-.75f);
        }
        else if (gamepad2.dpad_down) {
            robot.joint.setPower(.75f);
            robot.joint2.setPower(.75f);
        }
        else {
            robot.joint.setPower(0);
            robot.joint2.setPower(0);
        }
    }

    void driveTrainSpeed(){
        if(gamepad1.left_bumper && !gamepad1.right_bumper) //Drivetrain Speed Controls
         speedFactor = .5; //Fast (left bumper)
       else if(gamepad1.right_bumper && !gamepad1.left_bumper)
          speedFactor = 0.125; //Slow (right bumper)
        else if(!gamepad1.right_bumper && !gamepad1.left_bumper)
          speedFactor = 0.25; //Default (middle speed)
    }
}
