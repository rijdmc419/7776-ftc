/**
 * CameraTestOp - a simple operating mode that tries to acquire an image from the
 * phone camera and get some data from the image
 * Created by phanau on 12/9/15.
 */

package org.firstinspires.ftc.teamcode._Test._Sensors;

import android.app.Activity;
import android.graphics.Bitmap;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.CameraLib;
import org.firstinspires.ftc.teamcode._Libs.RS_Posterize;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;


@Autonomous(name="Test: Camera Test Vf RS", group ="Test")
//@Disabled
public class CameraTestOpVfRS extends OpMode {

    int mLoopCount;
    VuforiaLib_FTC2017 mVLib;
    ImageView mView;
    RS_Posterize mRsPosterize;
    Bitmap mBmOut;

    // Constructor
    public CameraTestOpVfRS() {
    }

    @Override
    public void init() {
        mVLib = new VuforiaLib_FTC2017();
        mVLib.init(this, null);     // pass it this OpMode (so it can do telemetry output) and use its license key for now

        mView = (ImageView)((Activity)hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.OpenCVOverlay);
        mView.post(new Runnable() {
            @Override
            public void run() {
                mView.setAlpha(1.0f);
            }
        });

    }

    @Override public void start()
    {
        // do this here -- takes too long to do in init()??
        mRsPosterize = new RS_Posterize();
        mRsPosterize.createScript(this.hardwareMap.appContext);

        /** Start tracking the data sets we care about. */
        mVLib.start();
    }

    public void loop() {

        // test image access through Vuforia
        Bitmap bmIn = mVLib.getBitmap(8);
        if (bmIn != null) {
            // create the output bitmap we'll display on the RC phone screen
            mBmOut = Bitmap.createBitmap(bmIn.getWidth(), bmIn.getHeight(), Bitmap.Config.RGB_565);

            // do some processing on the input bitmap in RenderScript to generate the output image
            mRsPosterize.runScript(bmIn, mBmOut);

            // do some data extraction on the processed bitmap
            /*
            CameraLib.CameraImage frame = new CameraLib.CameraImage(bmIn);
            CameraLib.Size camSize = frame.cameraSize();
            telemetry.addData("Size", String.valueOf(camSize.width) + "x" + String.valueOf(camSize.height));
            final int bandSize = 2;         // width of each band of pixel columns below
            final float minFrac = 0.4f;     // minimum fraction of pixels in band that needs to be of same color to mark it as "dominant"
            telemetry.addData("hue columns", frame.columnHue(bandSize, null, minFrac));
            telemetry.addData("dom columns", frame.columnDomColor(bandSize, null, minFrac));
            */

            //display the processed bitmap
            mView.post(new Runnable() {
                @Override
                public void run() {
                    //synchronized (bmLock) {
                    mView.setImageBitmap(mBmOut);
                    mView.invalidate();
                    //}
                }
            });
        }
    }

    public void stop() {
        mRsPosterize.destroyScript();
        mVLib.stop();
    }

}

