package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._Libs.PixyCamLego;

// test PixyCam using LEGO I2C protocol -- only returns biggest block of each signature

@TeleOp(name="TestPixyCamLego")
public class TestPixyCamLego extends OpMode
{

    PixyCamLego pixyCam;
    ElapsedTime elapsedTime = new ElapsedTime();

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init()
    {

        try {
            pixyCam = hardwareMap.get(PixyCamLego.class, "pixycamlego");
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
            PixyCamLego.Block block1 = pixyCam.GetBiggestBlock(1);
            telemetry.addData("Block 1:", block1.toString());
            PixyCamLego.Block block2 = pixyCam.GetBiggestBlock(2);
            telemetry.addData("Block 2:", block2.toString());
            PixyCamLego.Block block3 = pixyCam.GetBiggestBlock(3);
            telemetry.addData("Block 3:", block3.toString());
            PixyCamLego.Block block4 = pixyCam.GetBiggestBlock(4);
            telemetry.addData("Block 4:", block4.toString());
        }
    }
}