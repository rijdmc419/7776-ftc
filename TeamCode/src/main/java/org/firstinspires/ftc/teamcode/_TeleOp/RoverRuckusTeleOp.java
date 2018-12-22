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

        powSet();
        telemetry();
    }

    void telemetry(){
        DecimalFormat printFormat = new DecimalFormat ("#.###");

        //telemetry.addData("Gamepad 1", gamepad1);
        //telemetry.addData("Gamepad 2", gamepad2);

        telemetry.addData("Speed Factor", printFormat.format(speedFactor));
        telemetry.addData("Left", printFormat.format(left));
        telemetry.addData("Right", printFormat.format(right));
    }

    void powSet() {
        robot.fl.setPower(left);
        robot.bl.setPower(left);
        robot.fr.setPower(right);
        robot.br.setPower(right);
    }

    void driveTrainSpeed(){
        if(gamepad1.left_bumper && !gamepad1.right_bumper) //Drivetrain Speed Controls
         speedFactor = 0.5;
       else if(gamepad1.right_bumper && !gamepad1.left_bumper)
          speedFactor = 0.05;
        else if(!gamepad1.right_bumper && !gamepad1.left_bumper)
          speedFactor = 0.275;
    }
}
