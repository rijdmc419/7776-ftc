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
    double left, right;
    double speedFactor;
    double jointSpeed;
    double jointSpeedFactor = 0.5;
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

        left = -Math.pow(gamepad1.left_stick_y, 3) * speedFactor;
        right = -Math.pow(gamepad1.right_stick_y, 3) * speedFactor;

        jointSpeed = -Math.pow(gamepad2.left_stick_y, 3)* jointSpeedFactor;

        extend();
        powSet();
        telemetry();
    }

    void extend(){
        if(gamepad2.left_trigger > 0){
            extendSpeed = gamepad2.left_trigger * extendSpeedFactor * -1;
        }
        else if(gamepad2.right_trigger > 0){
            extendSpeed = gamepad2.right_trigger * extendSpeedFactor;
        }
        else{
            extendSpeed = 0;
        }
    }

    void telemetry(){
        DecimalFormat printFormat = new DecimalFormat ("#.###");

        //telemetry.addData("Gamepad 1", gamepad1);
        //telemetry.addData("Gamepad 2", gamepad2);

        telemetry.addData("Extend", printFormat.format(extendSpeed));
        telemetry.addData("Joint", printFormat.format(jointSpeed));
        telemetry.addData("Speed Factor", printFormat.format(speedFactor));
        telemetry.addData("Left", printFormat.format(left));
        telemetry.addData("Right", printFormat.format(right));
    }

    void powSet() {
        robot.fl.setPower(left);
        robot.bl.setPower(left);
        robot.fr.setPower(right);
        robot.br.setPower(right);
        robot.joint.setPower(jointSpeed);
        robot.joint2.setPower(jointSpeed);
        robot.extend.setPower(extendSpeed);
        robot.extend2.setPower(extendSpeed);
    }

    void driveTrainSpeed(){
        if(gamepad1.left_bumper && !gamepad1.right_bumper) //Drivetrain Speed Controls
         speedFactor = 1;
       else if(gamepad1.right_bumper && !gamepad1.left_bumper)
          speedFactor = 0.5;
        else if(!gamepad1.right_bumper && !gamepad1.left_bumper)
          speedFactor = 0.75;
    }
}
