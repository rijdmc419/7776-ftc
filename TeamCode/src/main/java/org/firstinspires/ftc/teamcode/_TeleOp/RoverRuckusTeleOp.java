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
    double liftSpeed = -.5;

   boolean brakeLast;
   boolean brake;

    @Override
    public void init() {
        robot.init(hardwareMap);
        brakeLast = false;
        brake = false;
    }

    @Override
    public  void start() {
       telemetry.addData("Lift Brake?", brake);
    }

    @Override
    public void loop() {
        driveTrainSpeed();
        liftBrake();

        left = -Math.pow(gamepad1.left_stick_y, 3) * speedFactor;
        right = -Math.pow(gamepad1.right_stick_y, 3) * speedFactor;

        if (brake == true)
            liftSpeed = -0.5;
        else
            liftSpeed =  -Math.pow(gamepad2.left_stick_y, 3);

        powSet();
        telemetry();
    }

    void telemetry(){
        DecimalFormat printFormat = new DecimalFormat ("#.###");

        //telemetry.addData("Gamepad 1", gamepad1);
        //telemetry.addData("Gamepad 2", gamepad2);
        telemetry.addData("Lift Pos",robot.lift.getCurrentPosition());
        telemetry.addData("Lift2 Pos",robot.lift2.getCurrentPosition());
        telemetry.addData("Speed Factor", printFormat.format(speedFactor));
        telemetry.addData("Left", printFormat.format(left));
        telemetry.addData("Right", printFormat.format(right));
        telemetry.addData("Lift Power", printFormat.format(liftSpeed));
        telemetry.addData("Lift Brake?", brake);
    }

    void powSet() {
        robot.fl.setPower(left);
        robot.bl.setPower(left);
        robot.fr.setPower(right);
        robot.br.setPower(right);
        robot.lift.setPower(liftSpeed);
        robot.lift2.setPower(liftSpeed);
    }

    void driveTrainSpeed(){
        if(gamepad1.left_bumper && !gamepad1.right_bumper) //Drivetrain Speed Controls
         speedFactor = 1;
       else if(gamepad1.right_bumper && !gamepad1.left_bumper)
          speedFactor = 0.1;
        else if(!gamepad1.right_bumper && !gamepad1.left_bumper)
          speedFactor = 0.55;
    }

    void liftBrake() {
            boolean brakePressed = gamepad2.a;

            if(brakePressed && !brakeLast){
               brake = !brake;
               if(brake) {
                  //while(robot.lift.getCurrentPosition() <= 540 && robot.lift2.getCurrentPosition() <= 540 && brake) {
                   liftSpeed = -0.5;
            //      }
               }
               else liftSpeed = Math.pow(gamepad2.left_stick_y, 3);
            }
            brakeLast = brakePressed;
    }

}
