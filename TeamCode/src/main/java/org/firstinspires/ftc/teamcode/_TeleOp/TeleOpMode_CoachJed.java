/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._Libs.SensorLib;

import java.util.Arrays;
import java.util.Collections;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpMode_CoachJed", group="Iterative Opmode")
public class TeleOpMode_CoachJed extends OpMode
{

    boolean bDebug = false;
    float max_controller = 1;
    // Set base wheel values
    float left_front_wheel = 0;
    float right_front_wheel = 0;
    float left_rear_wheel = 0;
    float right_rear_wheel = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private ModernRoboticsI2cGyro gyro_pointer;
    ModernRoboticsI2cGyro mGyro;            // gyro to use for heading information
    SensorLib.CorrectedMRGyro mCorrGyro;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        try {
            motorFrontRight = hardwareMap.dcMotor.get("frontRight");
            motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
            motorBackRight = hardwareMap.dcMotor.get("backRight");
            motorBackLeft = hardwareMap.dcMotor.get("backLeft");
            gyro_pointer = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("base_gyro");
            //motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
            //motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (IllegalArgumentException iax) {
            bDebug = true;
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (!bDebug) {
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // mechanum drive
        // note that if y equals -1 then joystick is pushed all of the way forward.
        float left_y = -gamepad1.left_stick_y;
        float left_x = gamepad1.left_stick_x;
        float right_x = gamepad1.right_stick_x;
//        float rotate_x = gamepad1.right_trigger - gamepad1.left_trigger;

        // Calculate the max value
        max_controller = Collections.max(Arrays.asList(max_controller, left_y, left_x, right_x));

        float inv_left_x = left_x * -1;
        float inv_right_x = right_x * -1;

        // Calculate the wheel Powers
        boolean all_zero = (abs(left_y) == 0)&& (abs(left_x) == 0) && (abs(right_x) == 0);
        float total_stick_abs = (Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x)) > max_controller ?
                (Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x)) : max_controller;// Allow variation?
        left_front_wheel = all_zero ? 0 : ((left_y+left_x+right_x)/(total_stick_abs*max_controller));
        left_rear_wheel = all_zero ? 0 : ((left_y+inv_left_x+right_x)/(total_stick_abs*max_controller));
        right_front_wheel = all_zero ? 0 : ((left_y+inv_left_x+inv_right_x)/(total_stick_abs*max_controller));
        right_rear_wheel = all_zero ? 0 : ((left_y+left_x+inv_right_x)/(total_stick_abs*max_controller));

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        left_front_wheel =  (float)scaleInput(left_front_wheel, max_controller);
        left_rear_wheel =  (float)scaleInput(left_rear_wheel, max_controller);
        right_front_wheel = (float)scaleInput(right_front_wheel, max_controller);
        right_rear_wheel = (float)scaleInput(right_rear_wheel, max_controller);

        // write the values to the motors - for now, front and back motors on each side are set the same
        if (!bDebug) {
            motorFrontRight.setPower(right_front_wheel);
            motorBackRight.setPower(right_rear_wheel);
            motorFrontLeft.setPower(left_front_wheel);
            motorBackLeft.setPower(left_rear_wheel);

//            // Set position of ball arm based on controller inputs
//            if(gamepad1.x) ballArm.setPosition(0.0f);
//            if(gamepad1.b) ballArm.setPosition(0.5f);
//            if(gamepad1.y) ballArm.setPosition(1.0f);

            // Control the ball lifter motor
//            ballLifter.setPower(rotate_x);
        }

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Jed Mech v1.0 ***");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("LJoystick Right Left", String.format("%.10f", left_x));
        telemetry.addData("LJoystick Forward Back", String.format("%.10f", left_y));
        telemetry.addData("RJoystick tLeft tRight", String.format("%.10f", right_x));
        telemetry.addData("Front Wheels: ", String.format("%.2f, %.2f", left_front_wheel, right_front_wheel));
        telemetry.addData("Back  Wheels: ", String.format("%.2f, %.2f", left_rear_wheel, right_rear_wheel));
//        telemetry.addData("LiftRotate: ", String.format("%.2f", rotate_x));
        telemetry.addData("gamepad1", gamepad1);
        telemetry.addData("gyro", gyro_pointer );
        // telemetry.addData("gamepad2", gamepad2);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(float dVal, float max_controller)  {
        return pow(dVal, 3)/pow(max_controller, 3);
    }

}
