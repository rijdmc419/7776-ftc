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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import static java.lang.Math.abs;

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

@TeleOp(name="TeleOpMode_Ori", group="Iterative Opmode")
public class TeleOpMode_Ori extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private Servo gripper = null;
    private boolean debug;
    private double gripperPos = 0;
    private double gripperChange = 0;

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
            leftfrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            rightfrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
            leftbackDrive = hardwareMap.get(DcMotor.class, "backLeft");
            rightbackDrive = hardwareMap.get(DcMotor.class, "backRight");
            gripper = hardwareMap.get(Servo.class, "gripper");

            leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
            //gripper.setPosition(0);
        }
        catch(IllegalArgumentException iax) {
            debug = true;
        }


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (!debug) {
            leftfrontDrive.setPower(0);
            rightfrontDrive.setPower(0);
            leftbackDrive.setPower(0);
            rightbackDrive.setPower(0);
        }
        gripperChange = 0;
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double powerY;
        double powerX;
        double powerLeft = 0;
        double powerRight = 0;
        double powerMax = 1;
        boolean padRight = false;
        boolean padLeft = false;
        double gripperIncrement = .01;

        powerY  = gamepad1.left_stick_y;
        powerX = -gamepad1.right_stick_x;
        padLeft = gamepad1.dpad_left;
        padRight = gamepad1.dpad_right;

        powerMax = Collections.max(Arrays.asList(powerMax, powerLeft, powerRight));

        boolean allZero = (abs(powerX) == 0) && (abs(powerY) == 0);
        double powerTotal = (abs(powerX) + abs(powerY)) > powerMax ? (abs(powerX) + abs(powerY)) : powerMax;

        powerLeft = allZero ? 0 : ((powerX + powerY) / (powerTotal * powerMax));
        powerLeft = -powerLeft;
        powerRight = allZero ? 0 : ((powerX - powerY) / (powerTotal * powerMax));

        if (padLeft) {
            gripperChange = -gripperIncrement;
        }

        else if (padRight) {
            gripperChange = gripperIncrement;
        }

        else {
            gripperChange = 0;
        }

        gripperPos = gripperPos + gripperChange;

        gripperPos = gripperPos > 1 ? 1 : gripperPos < 0 ? 0 : gripperPos;

        if (!debug) {
            // Send calculated power to wheels
            leftfrontDrive.setPower(powerLeft);
            rightfrontDrive.setPower(powerRight);
            leftbackDrive.setPower(powerLeft);
            rightbackDrive.setPower(powerRight);
            gripper.setPosition(gripperPos);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", powerLeft, powerRight);
        telemetry.addData("padRight", padRight);
        telemetry.addData("padLeft", padLeft);
        telemetry.addData("gripperPos", gripperPos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
