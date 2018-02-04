package org.firstinspires.ftc.teamcode._Test._Sensors;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.PX4Flow;


/**
 * Created by phanau on 2/4/18.
 * Optical flow sensor hardware test
 */
@Autonomous(name="Test: PX4Flow Test 1", group ="Test")
//@Disabled
public class PX4FlowTestOp extends OpMode {

    private PX4Flow mSensor;
    private View mRelativeLayout;

    public PX4FlowTestOp() {
    }

    public void init() {
        mSensor = new PX4Flow(this);
        mSensor.init();
    }

    public void loop() {
        // read current integrated data from sensor
        mSensor.readIntegral();

        // log data to DriverStation
        telemetry.addData("count", mSensor.frame_count_since_last_readout());
        telemetry.addData("dx", mSensor.pixel_flow_x_integral());
        telemetry.addData("dy", mSensor.pixel_flow_y_integral());
        telemetry.addData("quality", mSensor.quality_integral());
    }

    public void stop() {}

}
