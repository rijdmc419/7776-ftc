/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode._Auto;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.RectF;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BlobFinder;
import org.firstinspires.ftc.teamcode._Libs.RS_Posterize;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.SetPosterizer;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;


/**
 * This OpMode uses a Step that uses images from the VuforiaLib_RoverRuckus library to determine
 * the position of the orange block and move toward it to knock it off its perch.
 */

interface SetBitmap {
    public void setBitmap(Bitmap bm);
}


// this is a guide step that uses camera image data to
// guide the robot to the indicated bin of the cryptobox
//
class GoToBlockGuideStep extends AutoLib.MotorGuideStep {

    VuforiaLib_RoverRuckus mVLib;
    OpMode mOpMode;

    final int minDoneCount = 5;         // require "done" test to succeed this many consecutive times
    int mDoneCount;

    Point mBmSize = null;

    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";

    AutoLib.MotorGuideStep mMotorGuideStep;     // step used to actually control the motors based on directives from this step and gyro input

    boolean mbLookForBlue;

    final float tanCameraHalfFOV = 28.0f/50.0f;       // horizontal half angle FOV of S5 camera is atan(28/50) or about 29.25 degrees

    public GoToBlockGuideStep(OpMode opMode, VuforiaLib_RoverRuckus VLib, AutoLib.MotorGuideStep motorGuideStep) {
        mOpMode = opMode;
        mVLib = VLib;
        mMotorGuideStep = motorGuideStep;
        mDoneCount = 0;
    }

    public void set(ArrayList<AutoLib.SetPower> motorSteps){
        mMotorGuideStep.set(motorSteps);
    }


    public boolean loop() {
        super.loop();

        float distance = -1;

        TFObjectDetector tfod = mVLib.getTfod();
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                mOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());

                Recognition goldMineral = null;          // detected block
                if (updatedRecognitions.size() >= 1) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineral = recognition;
                        }
                    }
                }

                // one time, get size of Vuforia image so we can compute error angle
                if (mBmSize == null)
                    mBmSize = mVLib.getBitmapSize();

                // if we have new data ...
                if (goldMineral != null && mBmSize != null) {
                    int goldMineralX = (int)(goldMineral.getRight()+goldMineral.getLeft())/2;
                    mOpMode.telemetry.addData("Gold Mineral X", goldMineralX);

                    // the target point should be in the middle of the image if we're on course -
                    // if not, correct our course to center it --
                    // compute fractional error = fraction of image offset of target from center = [-1 .. +1]
                    // get size of Vuforia image
                    int bmWidth = mBmSize.x;
                    mOpMode.telemetry.addData("bitmap width", bmWidth);
                    float error = ((float) goldMineralX - (float) bmWidth / 2.0f) / ((float) bmWidth / 2.0f);

                    float blockSize = goldMineral.getRight()-goldMineral.getLeft();
                        // width is best measure of edge length of block in pixels since height is frame-limited
                    float viewFrac = blockSize/bmWidth;
                    distance = 2.0f / (viewFrac*2*tanCameraHalfFOV);            // distance to block given it is 2" wide;

                    // compute motor correction from error through PID --
                    // for now, convert image-pixel error to angle in degrees --
                    // image coordinates are positive to the right but angles are positive to the left (CCW)
                    float angError = (float) (Math.atan(error * tanCameraHalfFOV) * 180.0 / Math.PI);
                    mOpMode.telemetry.addData("data", "error=%f angError=%f", error, angError);

                    // tell the subsidiary motor guide step which way to steer -- the heading (orientation) will either be
                    // constant (squirrely wheels) or change to match the direction, depending on the motor guide step we're given.
                    ((AutoLib.SetDirectionHeadingPower) mMotorGuideStep).setRelativeDirection(angError);
                }
            }

        }

        // when we're really close ...
        if (distance > 0) {             // have valid distance data ...
            mOpMode.telemetry.addData("distance (in)", distance);
            if (distance < 16) {          // for now, complete this step at this distance from block
                // require completion test to pass some min number of times in a row to believe it
                mDoneCount++;
                if (mDoneCount >= minDoneCount) {
                    // return "done" -- assume the next step will stop motors if needed
                    return true;
                }
            } else
                mDoneCount = 0;         // reset the "done" counter
        }
        mOpMode.telemetry.addData("data", "doneCount=%d", mDoneCount);

        return false;  // haven't found anything yet
    }

    public void stop() {
    }

}


// for now, run this directly -- later this might be subordinated to red and blue OpModes
@Autonomous(name="RoverRuckusGoToBlockTF", group ="Auto")
//@Disabled
public class RoverRuckusGoToBlockTF extends OpMode  {

    boolean bDone;
    AutoLib.Sequence mSequence;             // the root of the sequence tree
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    VuforiaLib_RoverRuckus mVLib;           // Vuforia wrapper object used by Steps
    Point mBmSize;                          // size of the image Vuforia would return as a bitmap
    AutoLib.MotorGuideStep mGuideStep;      // member variable so that init() can set it and start() can use it
    double mTime;   // time of last loop() call -- used to compute frames per second


    boolean getHardware(AutoLib.HardwareFactory mf, DcMotor[] motors) {
        // get the motors: depending on the factory we are given, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        try {
            motors[0] = mf.getDcMotor("fr");
            motors[1] = mf.getDcMotor("br");
            motors[2] = mf.getDcMotor("fl");
            motors[3] = mf.getDcMotor("bl");
            return (motors[0]!=null && motors[1]!=null && motors[2]!=null && motors[3]!=null);
        }
        catch (Exception c) {
            return false;
        }
    }

    public void init() {
        init(false);    // for now, red/blue doesn't matter
    }

    public void init(boolean bLookForBlue)
    {
        // get the hardware
        mMotors = new DcMotor[4];
        if (!getHardware(new AutoLib.RealHardwareFactory(this), mMotors)) {
            getHardware(new AutoLib.TestHardwareFactory(this), mMotors);
        }

        boolean invertLeft = true;     // current ratbot ...
        if (invertLeft) {
            mMotors[2].setDirection(DcMotor.Direction.REVERSE);
            mMotors[3].setDirection(DcMotor.Direction.REVERSE);
        }
        else {
            mMotors[0].setDirection(DcMotor.Direction.REVERSE);
            mMotors[1].setDirection(DcMotor.Direction.REVERSE);
        }

        // best to do this now, which is called from opmode's init() function
        mVLib = new VuforiaLib_RoverRuckus();
        mVLib.init(this, null);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();
        // make a step that will steer the robot given guidance from the GoToBlockGuideStep

        // construct a PID controller for correcting heading errors
        final float Kp = 0.005f;        // degree heading proportional term correction per degree of deviation
        final float Ki = 0.005f;        // ... integrator term
        final float Kd = 0.0f;         // ... derivative term
        final float KiCutoff = 0.0f;   // maximum angle error for which we update integrator
        SensorLib.PID pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        final float power = 0.2f;
        AutoLib.MotorGuideStep motorGuideStep = new AutoLib.ErrorGuideStep(this, pid, null, power);
        // make a step that analyzes a camera image from Vuforia and steers toward the biggest orange blob, presumably the block.
        // it uses a ErrorGuideStep to process the heading error it computes into motor steering commands.
        mGuideStep  = new GoToBlockGuideStep(this, mVLib, motorGuideStep);
        // make and add the Step that combines all of the above to go to the orange block
        mSequence.add(new AutoLib.GuidedTerminatedDriveStep(this, mGuideStep, motorGuideStep, mMotors));
        // continue on unguided for 1 sec and then stop all motors
        mSequence.add(new AutoLib.MoveByTimeStep(mMotors, power, 2,true));
    }

    @Override public void start()
    {
        // start out not-done
        bDone = false;

        // start Vuforia scanning
        mVLib.start();

        /** Activate Tensor Flow Object Detection. */
        TFObjectDetector tfod = mVLib.initTfod(this);
        if (tfod != null) {
            tfod.activate();
        }

        // get start time for frames/sec computation
        mTime = getRuntime();
    }

    @Override
    public void loop() {
        // log frames per second
        double dTime = getRuntime() - mTime;
        if (dTime > 0)
            telemetry.addData("frames/sec", 1.0/dTime);
        mTime = getRuntime();       // for next time ...

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
    }

}

