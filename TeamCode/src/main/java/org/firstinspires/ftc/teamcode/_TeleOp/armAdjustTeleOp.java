package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

@TeleOp(name="Manual Arm Adjust")
public class armAdjustTeleOp  extends OpMode {
    RoverRuckusHardware robot =  new RoverRuckusHardware();
    double left, right;

    @Override
    public void init(){
        robot.init(hardwareMap);
    }

    @Override
    public void loop(){
        left = -Math.pow(gamepad1.left_stick_y, 3);
        right = -Math.pow(gamepad1.left_stick_y, 3);

        robot.extend.setPower(left);
        robot.extend2.setPower(right);
    }
}
