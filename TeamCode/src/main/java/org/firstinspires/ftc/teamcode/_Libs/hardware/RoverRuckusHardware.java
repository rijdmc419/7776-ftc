package org.firstinspires.ftc.teamcode._Libs.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by bremm on 10/26/18.
 */

public class RoverRuckusHardware {
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor fr  = null;
    public DcMotor br   = null;
    public DcMotor lift = null;
    public DcMotor lift2 = null;

    public Servo markerServo;

    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap) {
        hwMap = ahwMap;

        fl = hwMap.get(DcMotor.class, "fl");
        bl = hwMap.get(DcMotor.class, "bl");
        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");

        lift = hwMap.get(DcMotor.class, "lift");
        lift2 = hwMap.get(DcMotor.class, "lift2");

        markerServo = hwMap.get(Servo.class, "markerServo");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.REVERSE);

        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        lift.setPower(0);
        lift2.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        markerServo.setPosition(-1);
    }
}
