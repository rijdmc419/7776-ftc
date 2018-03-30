package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode._Libs.PixyCam;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.R.attr.rotation;

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
        pixyCam = hardwareMap.get(PixyCam.class, "pixycam");
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop()
    {
        // if (elapsedTime.milliseconds() > 33) // Update every 1/30 of a second.
        {
            telemetry.addData("dt(millisec):", elapsedTime.milliseconds());
            elapsedTime.reset();
            PixyCam.Block block1 = pixyCam.GetBiggestBlock(1);
            telemetry.addData("Block 1:", block1.toString());
            PixyCam.Block block2 = pixyCam.GetBiggestBlock(2);
            telemetry.addData("Block 2:", block2.toString());
            PixyCam.Block block3 = pixyCam.GetBiggestBlock(3);
            telemetry.addData("Block 3:", block3.toString());
            PixyCam.Block block4 = pixyCam.GetBiggestBlock(4);
            telemetry.addData("Block 4:", block4.toString());
        }
    }
}