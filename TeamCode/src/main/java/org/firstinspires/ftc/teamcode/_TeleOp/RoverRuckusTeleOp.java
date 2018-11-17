package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

import java.text.DecimalFormat;

/**
 * Created by bremm on 10/26/18.
 */

@TeleOp(name="Rover Ruckus TeleOp")



public class RoverRuckusTeleOp extends OpMode{
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    double left, right;
    double speedFactor = 0.5;
    double liftSpeed = 1;
    int brakeCount = 0;
    boolean liftBrake = false;
    DecimalFormat printFormat = new DecimalFormat ("#.###");
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper && !gamepad1.right_bumper) //Drivetrain Speed Controls
            speedFactor = 1;
        else if(gamepad1.right_bumper && !gamepad1.left_bumper)
            speedFactor = 0.25;
        else if(!gamepad1.right_bumper && !gamepad1.left_bumper)
            speedFactor = 0.5;

        if(gamepad2.left_bumper && !gamepad2.right_bumper) {//Lift Controls
            robot.lift.setPower(liftSpeed);
            robot.lift2.setPower(liftSpeed);}
        else if(gamepad2.right_bumper && !gamepad2.left_bumper){
        robot.lift.setPower(-1);
        robot.lift2.setPower(-1 * liftSpeed); }
        else if(!gamepad2.right_bumper && !gamepad2.left_bumper){
            robot.lift.setPower(0);
            robot.lift2.setPower(0); }

        if(gamepad2.b) {//Lift Controls
            robot.lift.setPower(0);
            robot.lift2.setPower(0);}



        left = (-1)* Math.pow(gamepad1.left_stick_y, 3) * speedFactor;
        right = (-1)* Math.pow(gamepad1.right_stick_y, 3) * speedFactor;

        robot.fl.setPower(left);
        robot.bl.setPower(left);
        robot.fr.setPower(right);
        robot.br.setPower(right);

        //telemetry.addData("Gamepad 1: ", gamepad1);
        //telemetry.addData("Gamepad 2:", gamepad2);
        telemetry.addData("Speed Factor: ", printFormat.format(speedFactor));
        telemetry.addData("Left: ", printFormat.format(left));
        telemetry.addData("Right: ", printFormat.format(right));
        telemetry.addData("Lift Motor 1: ", printFormat.format(robot.lift.getPower()));
        telemetry.addData("Lift Motor 2: ", printFormat.format(robot.lift2.getPower()));
        telemetry.addData("Lift Braked: ", gamepad2.b);

    }
}
