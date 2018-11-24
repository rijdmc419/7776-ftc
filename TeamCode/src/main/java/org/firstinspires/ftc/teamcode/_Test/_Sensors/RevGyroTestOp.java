package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;

import static android.os.SystemClock.sleep;


/**
 * Created by phanau on 1/22/16.
 * Test REV Robotics RevHub IMU
 */

@Autonomous(name="Test: REV IMU Test 1", group ="Test")
//@Disabled
public class RevGyroTestOp extends OpMode {

    private BNO055IMUHeadingSensor mIMU;

    public RevGyroTestOp() {
    }

    public void init() {
        // get hardware IMU and wrap gyro in HeadingSensor object usable below
        mIMU = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mIMU.init(3);  // 3: upright crosswise with REV face forward
    }

    public void loop() {
        telemetry.addData("orientation: ", mIMU.getOrientation());
        telemetry.addData("position: ", mIMU.getPosition());
    }

    // the default orientation of the IMU axes in the REV hub is like this:
    //       __________________________
    //      |                          |
    //      |  REV                     |
    //      |           Z@------ Y     |
    //      |            |             |        ---------> forward
    //      |            |             |
    //      |            X             |
    //      |__________________________|

    // we will use the convention PITCH = +X, ROLL = +Y, YAW(Heading) = +Z
    // which corresponds to mounting the REV hub flat on the vehicle with the front in the +Y direction.

    // this function sets CONFIG and SIGN bytes for REV mounted in various other orientations
    private void setRevOrientation(BNO055IMU imu) {
        // other orientations need these settings:
        // * flat with R in REV closest to front of vehicle:
        // byte AXIS_MAP_CONFIG_BYTE = 0x24;    // Z=Z Y=-Y X=-X
        // byte AXIS_MAP_SIGN_BYTE = 0x6;       // -X -Y Z
        // * upright longitudinally with front of vehicle nearest the R
        // byte AXIS_MAP_CONFIG_BYTE = 0x6;     // Z=-X Y=-Y X=-Z
        // byte AXIS_MAP_SIGN_BYTE = 0x1;       // X Y -Z     ?? if 0x7 -X -Y -Z, X and Y are LH rotations
        // * upright crosswise with REV face forward
        byte AXIS_MAP_CONFIG_BYTE = 0x9;        // Z=-X Y=Z X=-Y
        byte AXIS_MAP_SIGN_BYTE = 0x3;          // X -Y -Z      ?? if 0x5 -X Y -Z, X and Y are LH rotations


        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal);
        sleep(19); //Changing to CONFIG mode require a delay of 19ms before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal);
        sleep(7); //Changing back to any operating mode requires a delay of 7ms
    }
}
