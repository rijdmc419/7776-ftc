/**
 * CameraTestOp - a simple operating mode that tries to acquire an image from the
 * phone camera and get some data from the image
 * Created by phanau on 12/9/15.
 */

package org.firstinspires.ftc.teamcode._Test._Sensors;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.BlobFinder;
import org.firstinspires.ftc.teamcode._Libs.CameraLib;
import org.firstinspires.ftc.teamcode._Libs.RS_Posterize;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;


@Autonomous(name="Test: Camera Test Vf RS", group ="Test")
//@Disabled
public class CameraTestOpVfRS extends OpMode {

    int mLoopCount;
    VuforiaLib_RoverRuckus mVLib;
    ImageView mView;
    RS_Posterize mRsPosterize;
    Bitmap mBmOut;
    Paint mPaintGreen;

    // Constructor
    public CameraTestOpVfRS() {
    }

    @Override
    public void init() {
        mVLib = new VuforiaLib_RoverRuckus();
        mVLib.init(this, null);     // pass it this OpMode (so it can do telemetry output) and use its license key for now

        mView = (ImageView)((Activity)hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.OpenCVOverlay);
        mView.post(new Runnable() {
            @Override
            public void run() {
                mView.setAlpha(1.0f);
            }
        });

        mPaintGreen = new Paint();
        mPaintGreen.setColor(Color.GREEN);
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
        Bitmap bmIn = mVLib.getBitmap(16);
        if (bmIn != null) {
            // create the output bitmap we'll display on the RC phone screen
            mBmOut = Bitmap.createBitmap(bmIn.getWidth(), bmIn.getHeight(), Bitmap.Config.RGB_565);

            // do some processing on the input bitmap in RenderScript to generate the output image
            mRsPosterize.runScript(bmIn, mBmOut);

            // do some data extraction on the processed bitmap
            CameraLib.CameraImage frame = new CameraLib.CameraImage(bmIn);
            CameraLib.Size camSize = frame.cameraSize();
            telemetry.addData("Size", String.valueOf(camSize.width) + "x" + String.valueOf(camSize.height));
            telemetry.addData("Center In", String.format("0x%08x", bmIn.getPixel(bmIn.getWidth()/2, bmIn.getHeight()/2)));
            telemetry.addData("Center Out", String.format("0x%08x", mBmOut.getPixel(mBmOut.getWidth()/2, mBmOut.getHeight()/2)));

            BlobFinder bf = new BlobFinder(mBmOut);
            int count = bf.find(0xFFFFFF00);    // posterized yellow value
            Point centroid = bf.getCentroid();
            telemetry.addData("blob count", count);
            telemetry.addData("centroid", centroid.x + "," + centroid.y);

            // add annotations to the bitmap showing detected column centers
            final int XS=5;     // cross size
            Canvas canvas = new Canvas(mBmOut);
            canvas.drawLine(centroid.x-XS, centroid.y, centroid.x+XS, centroid.y, mPaintGreen);
            canvas.drawLine(centroid.x, centroid.y-XS, centroid.x, centroid.y+XS, mPaintGreen);

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
        if (mRsPosterize != null)
            mRsPosterize.destroyScript();
        mVLib.stop();       // Vuforia claims to do this automatically on OpMode stop but ...
        mView.post(new Runnable() {
            @Override
            public void run() {
                mView.setAlpha(0.0f);
            }
        });     // hide the overlay window
    }

}

