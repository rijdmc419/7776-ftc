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
package org.firstinspires.ftc.teamcode._Test._AutoLib;

import android.graphics.Bitmap;
import android.graphics.Point;
import android.graphics.Rect;
import android.graphics.RectF;
import android.hardware.Camera;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.CameraLib;
import org.firstinspires.ftc.teamcode._Libs.DistanceSensor;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This OpMode uses a Step that uses the VuforiaLib_FTC2017 library to determine
 * which column of the shelves to fill first, then
 * moves the robot under gyro control while using the camera to look for the
 * correct Cryptobox to stop at.
 */

// define an interface through which a Step (or anything else) can be told the
// identity of the Vuforia target that we should use
interface SetMark {
    public void setMark(String s);
}

class VuforiaGetMarkStep extends AutoLib.Step {

    VuforiaLib_FTC2017 mVLib;
    OpMode mOpMode;
    SetMark mSMStep;

    public VuforiaGetMarkStep(OpMode opMode, VuforiaLib_FTC2017 VLib, SetMark step) {
        mOpMode = opMode;
        mVLib = VLib;
        mSMStep = step;
    }

    public boolean loop() {
        super.loop();
        mVLib.loop();       // update recognition info
        RelicRecoveryVuMark vuMark = mVLib.getVuMark();
        boolean found = (vuMark != RelicRecoveryVuMark.UNKNOWN);
        if (found) {
            // Found an instance of the template -- tell "MoveTo.. step which one
            mSMStep.setMark(vuMark.toString());
        }
        return found;       // done?
    }
}

class BlueFilter implements CameraLib.Filter {
    public int map(int hue) {
        // map 4 (cyan) to 5 (blue)
        if (hue == 4)
            return 5;
        else
            return hue;
    }
}

// simple data class containing info about image of one column of cryptobox
class ColumnHit {
    int mStart;
    int mEnd;
    public ColumnHit(int start, int end) {
        mStart = start;  mEnd = end;
    }
    public int start() { return mStart; }
    public int end() { return mEnd; }
    public int mid() { return (mStart+mEnd)/2; }
}

// this is a guide step that uses camera image data to
// guide the robot to the indicated bin of the cryptobox
//
class GoToCryptoBoxGuideStep extends AutoLib.MotorGuideStep implements SetMark {

    VuforiaLib_FTC2017 mVLib;
    String mVuMarkString;
    OpMode mOpMode;
    int mCBColumn;                      // which Cryptobox column we're looking for
    Pattern mPattern;                   // compiled regexp pattern we'll use to find the pattern we're looking for
    int mColumnOffset;                  // number of columns that have left the left-edge of the frame

    CameraLib.Filter mBlueFilter;       // filter to map cyan to blue

    ArrayList<ColumnHit> mPrevColumns;  // detected columns on previous pass

    SensorLib.PID mPid;                 // proportional–integral–derivative controller (PID controller)
    double mPrevTime;                   // time of previous loop() call
    ArrayList<AutoLib.SetPower> mMotorSteps;   // the motor steps we're guiding - assumed order is right ... left ...
    float mPower;                      // base power setting for motors

    public GoToCryptoBoxGuideStep(OpMode opMode, VuforiaLib_FTC2017 VLib, String pattern, float power) {
        mOpMode = opMode;
        mCBColumn = 1;     // if we never get a cryptobox directive from Vuforia, go for the first bin
        mPattern = Pattern.compile(pattern);    // look for the given pattern of column colors
        mBlueFilter = new BlueFilter();
        mVLib = VLib;
        mMotorSteps = null;     // this will be filled in by call from parent step
        mPower = power;
        mPrevColumns = null;
        mColumnOffset = 0;

        // construct a default PID controller for correcting heading errors
        final float Kp = 0.2f;         // degree heading proportional term correction per degree of deviation
        final float Ki = 0.0f;         // ... integrator term
        final float Kd = 0.0f;         // ... derivative term
        final float KiCutoff = 3.0f;   // maximum angle error for which we update integrator
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

    }

    public void setMark(String s) {
        mVuMarkString = s;

        // compute index of column that forms the left side of the desired bin.
        // this assumes the camera is mounted to the left of the carried block.
        if (mVuMarkString == "LEFT")
            mCBColumn = 0;
        else
            if (mVuMarkString == "CENTER")
                mCBColumn = 1;
        else
            if (mVuMarkString == "RIGHT")
                mCBColumn = 2;

        // if the camera is on the right side of the block, we want the right edge of the bin.
        final boolean bCameraOnRight = true;
        if (bCameraOnRight)
            mCBColumn++;
    }

    public void set(ArrayList<AutoLib.SetPower> motorSteps){
        mMotorSteps = motorSteps;
    }

    public boolean loop() {
        super.loop();

        final int minDoneCount = 5;      // require "done" test to succeed this many consecutive times
        int doneCount = 0;

        // initialize previous-time on our first call -> dt will be zero on first call
        if (firstLoopCall()) {
            mPrevTime = mOpMode.getRuntime();           // use timer provided by OpMode
        }

        mOpMode.telemetry.addData("VuMark", "%s found", mVuMarkString);

        // get most recent frame from camera (through Vuforia)
        //RectF rect = new RectF(0,0.25f,1f,0.75f);      // middle half of the image should be enough
        //Bitmap bitmap = mVLib.getBitmap(rect, 4);                      // get cropped, downsampled image from Vuforia
        Bitmap bitmap = mVLib.getBitmap(8);                      // get uncropped, downsampled image from Vuforia
        CameraLib.CameraImage frame = new CameraLib.CameraImage(bitmap);       // .. and wrap it in a CameraImage

        if (bitmap != null && frame != null) {
            // look for cryptobox columns
            // get unfiltered view of colors (hues) by full-image-height column bands
            final int bandSize = 4;
            String colString = frame.columnHue(bandSize);

            // log debug info ...
            mOpMode.telemetry.addData("hue columns", colString);

            // look for occurrences of given pattern of column colors
            ArrayList<ColumnHit> columns = new ArrayList<ColumnHit>(8);       // array of column start/end indices

            for (int i=0; i<colString.length(); i++) {
                // starting at position (i), look for the given pattern in the encoded (rgbcymw) scanline
                Matcher m = mPattern.matcher(colString.substring(i));
                if (m.lookingAt()) {
                    // add start/end info about this hit to the array
                    columns.add(new ColumnHit(i+m.start(), i+m.end()-1));

                    // skip over this match
                    i += m.end();
                }
            }

            // report the matches in telemetry
            for (ColumnHit h : columns) {
                mOpMode.telemetry.addData("found ", "%s from %d to %d", mPattern.pattern(), h.start(), h.end());
            }

            int nCol = columns.size();

            // compute average distance between columns = distance between outermost / #bins
            float avgBinWidth = nCol>1 ? (float)(columns.get(nCol-1).end() - columns.get(0).start()) / (float)(nCol-1) : 0;

            // try to handle case where a column has left (or entered) the left-edge of the frame between the prev view and this one
            if (mPrevColumns != null  &&  mPrevColumns.size()>0  &&  nCol>0  && avgBinWidth > 0) {

                // if the left-most column of the previous frame started at the left edge of the frame
                // and the left edge of the current left-most column is about a bin-width to the right of the right edge of the
                // left-most column of the previous frame
                // then it's probably the case that the current left-most column is actually the second column of the previous frame.
                if (mPrevColumns.get(0).start() == 0 && columns.get(0).start() > (mPrevColumns.get(0).end()+0.5*avgBinWidth)) {
                    mColumnOffset++;
                }

                // if the left-most column of the previous frame was not near the left edge of the frame
                // but now there is a column at the left edge, then one probably entered the frame.
                if (mColumnOffset > 0 && mPrevColumns.get(0).start() > 0.5*avgBinWidth  &&  columns.get(0).start() == 0) {
                    mColumnOffset--;
                }
            }

            mOpMode.telemetry.addData("data", "avgWidth= %f  mColOff=%d", avgBinWidth, mColumnOffset);

            // if we found some columns, try to correct course using their positions in the image
            if (mCBColumn >= mColumnOffset && nCol > mCBColumn-mColumnOffset) {
                // to start, we need to see all four columns to know where we're going ...
                // after that, we try to match up the columns visible in this view with those from the previous pass
                // TBD

                // compute camera offset from near-side column of target bin (whichever side camera is to the block holder)
                final float cameraOffset = 0.2f;        // e.g. camera is 0.2 x bin width to the right of block centerline
                float cameraBinOffset = avgBinWidth * cameraOffset;
                // camera target is center of target column + camera offset in image-string space
                float cameraTarget = columns.get(mCBColumn-mColumnOffset).mid() + cameraBinOffset;

                // the above computed target point should be in the middle of the image if we're on course -
                // if not, correct our course to center it --
                // compute fractional error = fraction of image offset of target from center = [-1 .. +1]
                float error = (cameraTarget - (float)colString.length()/2.0f) / ((float)colString.length()/2.0f);

                // compute motor correction from error through PID --
                // for now, convert image-string error to angle and use standard "gyro" PID
                final float cameraHalfFOVdeg = 28.0f;       // half angle FOV is about 28 degrees
                float angError = error * cameraHalfFOVdeg;

                mOpMode.telemetry.addData("data", "target=%f  error=%f angError=%f", cameraTarget, error, angError);

                // compute delta time since last call -- used for integration time of PID step
                double time = mOpMode.getRuntime();
                double dt = time - mPrevTime;
                mPrevTime = time;

                // feed error through PID to get motor power correction value
                float correction = -mPid.loop(error, (float)dt);

                // compute new right/left motor powers
                float rightPower = mPower + correction;
                float leftPower = mPower - correction;

                // normalize so neither has magnitude > 1
                float norm = AutoLib.normalize(rightPower, leftPower);
                rightPower *= norm;
                leftPower *= norm;

                // set the motor powers -- handle both time-based and encoder-based motor Steps
                // assumed order is right motors followed by an equal number of left motors
                int i = 0;
                for (AutoLib.SetPower ms : mMotorSteps) {
                    ms.setPower((i++ < mMotorSteps.size()/2) ? rightPower : leftPower);
                }

                mOpMode.telemetry.addData("motors", "left=%f right=%f", leftPower, rightPower);
            }

            // when we're really close ... i.e. when the bin width is really big ... we're done
            if (nCol > 1 && avgBinWidth > colString.length()/2) {          // for now, when bin width > 1/2 FOV
                // require completion test to pass some min number of times in a row to believe it
                if (++doneCount >= minDoneCount) {
                    // stop all the motors and return "done"
                    for (AutoLib.SetPower ms : mMotorSteps) {
                        ms.setPower(0.0);
                    }
                    return true;
                }
            }
            else
                doneCount = 0;         // reset the "done" counter

            mOpMode.telemetry.addData("data", "doneCount=%d", doneCount);

            // save column hits for next pass to help handle columns leaving the field of view of
            // the camera as we get close.
            mPrevColumns = columns;

        }

        return false;  // haven't found anything yet
    }

    public void stop() {
    }
}


//@Autonomous(name="FirstRelicRecAuto1", group ="Auto")
//@Disabled
public class FirstRelicRecAuto1 extends OpMode {

    boolean bDone;
    AutoLib.Sequence mSequence;             // the root of the sequence tree
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    GyroSensor mGyro;                       // gyro to use for heading information
    SensorLib.CorrectedGyro mCorrGyro;      // gyro corrector object
    VuforiaLib_FTC2017 mVLib;               // Vuforia wrapper object used by Steps

    public void init() {}

    public void init(boolean bLookForBlue)
    {
        // get the hardware
        AutoLib.HardwareFactory mf = null;
        final boolean debug = true;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        mMotors = new DcMotor[4];
        mMotors[0] = mf.getDcMotor("fr");
        mMotors[1] = mf.getDcMotor("br");
        (mMotors[2] = mf.getDcMotor("fl")).setDirection(DcMotor.Direction.REVERSE);
        (mMotors[3] = mf.getDcMotor("bl")).setDirection(DcMotor.Direction.REVERSE);

        // get hardware gyro
        mGyro = mf.getGyro("gyro");

        // wrap gyro in an object that calibrates it and corrects its output
        mCorrGyro = new SensorLib.CorrectedGyro(mGyro);
        mCorrGyro.calibrate();

        // best to do this now, which is called from opmode's init() function
        mVLib = new VuforiaLib_FTC2017();
        mVLib.init(this, null);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();
        // make a step that guides the motion step by looking for a particular (red or blue) Cryptobox
        // it also implements the SetMark interface so VuforiaGetMarkStep can call it to tell which box to go for
        AutoLib.MotorGuideStep guideStep  = new GoToCryptoBoxGuideStep(this, mVLib, bLookForBlue ? "^b+" : "^r+", 0.6f);
        // make and add to the sequence the step that looks for the Vuforia marker and sets the column (Left,Center,Right)
        // the motion terminator step should look for
        mSequence.add(new VuforiaGetMarkStep(this, mVLib, (SetMark)guideStep));
        // make and add the Step that goes to the indicated Cryptobox bin
        mSequence.add(new AutoLib.GuidedTerminatedDriveStep(this, guideStep, null, mMotors));
        // make and add a step that tells us we're done
        mSequence.add(new AutoLib.LogTimeStep(this,"Done!", 5));
    }

    @Override public void start()
    {
        // start out not-done
        bDone = false;

        // start Vuforia scanning
        mVLib.start();
    }

    @Override
    public void loop() {

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
        mVLib.stop();     // make sure the Camera is released by Vuforia
    }

}

