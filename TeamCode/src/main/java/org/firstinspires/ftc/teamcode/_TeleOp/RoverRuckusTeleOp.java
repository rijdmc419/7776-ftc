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

    double speedFactor;
    double liftSpeed = 1;
    DecimalFormat printFormat = new DecimalFormat ("#.###");

   // boolean brakeLast;
   // boolean brake;

    @Override
    public void init() {
        robot.init(hardwareMap);
      //  brakeLast = false;
    //    brake = true;
    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper && !gamepad1.right_bumper) //Drivetrain Speed Controls
            speedFactor = 1;
        else if(gamepad1.right_bumper && !gamepad1.left_bumper)
            speedFactor = 0.1;
        else if(!gamepad1.right_bumper && !gamepad1.left_bumper)
            speedFactor = 0.55;

        if(gamepad2.b) {
            robot.lift.setPower(1);
            robot.lift2.setPower(1);
        }

        //boolean brakePressed = gamepad2.b;
        //if(brakePressed && !brakeLast){
         //   brake = !brake;
       //     if(brake) liftSpeed = 1;
         //   else liftSpeed =
       // }
    //    brakeLast = brakePressed;

        left = (-1)* Math.pow(gamepad1.left_stick_y, 3) * speedFactor;
        right = (-1)* Math.pow(gamepad1.right_stick_y, 3) * speedFactor;
        liftSpeed =  Math.pow(gamepad2.right_stick_y, 3);

        robot.fl.setPower(left);
        robot.bl.setPower(left);
        robot.fr.setPower(right);
        robot.br.setPower(right);
        robot.lift.setPower(liftSpeed);
        robot.lift2.setPower(liftSpeed);

        //telemetry.addData("Gamepad 1: ", gamepad1);
        //telemetry.addData("Gamepad 2:", gamepad2);
        telemetry.addData("Speed Factor: ", printFormat.format(speedFactor));
        telemetry.addData("Left: ", printFormat.format(left));
        telemetry.addData("Right: ", printFormat.format(right));
        telemetry.addData("Lift Power: ", printFormat.format(liftSpeed));
      //  telemetry.addData("Lift Brake? ", brake);
    }
}
