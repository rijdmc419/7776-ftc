package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode._Libs.PixyCam;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import static android.R.attr.rotation;

// test program for PixyCam running normal streaming I2C protocol
// which should return multiple blocks of the same color if that's what's seen.
// TBD -- rewrite this code to actually do that ...

@TeleOp(name="TestPixyCam")
public class TestPixyCam extends OpMode
{

    PixyCam pixyCam;
    ElapsedTime elapsedTime = new ElapsedTime();

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init()
    {

        try {
            pixyCam = hardwareMap.get(PixyCam.class, "pixycam");
        }
        catch (Exception e) {
            pixyCam = null;
        }
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop()
    {
        // if (elapsedTime.milliseconds() > 33) // Update every 1/30 of a second.
        if (pixyCam == null) {
            telemetry.addData("no Pixycam available","");
        }
        else {
            telemetry.addData("dt(millisec):", elapsedTime.milliseconds());
            elapsedTime.reset();
            int count = pixyCam.getBlocks(100);
            telemetry.addData("numBlocks or error code", count);
            ArrayList<PixyCam.Block> blocks = pixyCam.getBlocks();
            for (PixyCam.Block b : blocks)
                telemetry.addData("Block", b.toString());
        }
    }
}