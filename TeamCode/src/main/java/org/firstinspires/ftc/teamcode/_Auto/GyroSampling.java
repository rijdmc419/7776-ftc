package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

import java.util.List;

public class GyroSampling extends AutoLib.Step {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manazger.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AW4D/s//////AAABmR179hwM4krTuTxhLhAFVGRrboS4DEQwwh7qDRneUEke4qw2wnQhvfwgBPUFOgyry9YTKwD7GPJOnLrU62RERSFShyC2+IZKL++AxcvYpPJeN0PcAh6LKPC+Nkh84MhCxoU5xfEhnyKgHDloFbRnc2jBEJPLf1qNQUXQwSleOGFPALA/MbCY0MIAkaYFP+/p3O98uRkkIt+vfDPJRTl4SP6VHBjNLsdAUjpg4iOCFMZyXC7zwMZIhZbwCVzxW/zvDJtwMtZZ8Zl4ye1lMvsIJslR9Ge05wMVcETqOP3z6skMIkXZkcoBotYdPa8QpBXjJBt1Ldgv3/eikBVhgKu7QfwQ3cDr5orgM2AlmWEC62YH";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    OpMode mOpMode;
    AutoLib.Timer mTimer;
    AutoLib.AzimuthTolerancedTurnStep mTurnStep;
    AutoLib.AzimuthTolerancedTurnStep mReturnStep;
    AutoLib.AzimuthTolerancedTurnStep mTurnToCraterStep;
    AutoLib.TurnByEncoderStep mDriveAfterTurn;
    AutoLib.TurnByEncoderStep mDriveBacktoOrigin;
    RoverRuckusHardware mRobot;
    float mGoldPositionAngle;
    int mGoldPosition;
    int mGyroAngle;

    public GyroSampling(OpMode opMode, AutoLib.AzimuthTolerancedTurnStep turnStep, RoverRuckusHardware robot, AutoLib.AzimuthTolerancedTurnStep returnStep,
                        AutoLib.AzimuthTolerancedTurnStep turnToCraterStep, AutoLib.TurnByEncoderStep driveAfterTurn, AutoLib.TurnByEncoderStep driveBacktoOrigin, int gyroAngle) {
        mOpMode = opMode;
        mTurnStep = turnStep;
        mReturnStep = returnStep;
        mTurnToCraterStep = turnToCraterStep;
        mDriveAfterTurn = driveAfterTurn;
        mDriveBacktoOrigin = driveBacktoOrigin;
        mRobot = robot;
        mGyroAngle = gyroAngle;
        mTimer = new AutoLib.Timer(3);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            mOpMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

    }

    public boolean loop() {
        super.loop();
        /** Activate Tensor Flow Object Detection. */
        if (firstLoopCall() && tfod != null) {
            tfod.activate();
            mTimer.start();
        }

        //one inch is 89.1 inches: c = d * counts per turn / circumference of wheel

        Recognition goldMineral = null;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                mOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() >= 1) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineral = recognition;
                            mGoldPositionAngle = (float) (goldMineral.estimateAngleToObject(AngleUnit.DEGREES));
                            mOpMode.telemetry.addData("Gold Position Angle", mGoldPositionAngle);
                            if(mGoldPositionAngle <= -15) {//left
                                mGoldPosition = 0;
                                mTurnStep.setHeading(30);
                                mReturnStep.setHeading(-45);
                                mDriveAfterTurn.set(-.25f, -.25f, -1575, -1575);
                                mDriveBacktoOrigin.set(-.25f, -.25f, 1575, 1575);
                                mTurnToCraterStep.setHeading(135);
                                tfod.shutdown();
                                return true;
                            }
                            else if (mGoldPositionAngle < 15) {//center
                                mGoldPosition = 1;
                                mTurnStep.setHeading(0);
                                mReturnStep.setHeading(0);
                                mDriveAfterTurn.set(-.25f, -.25f, -1215, -1215);
                                mDriveBacktoOrigin.set(-.25f, -.25f, 1215, 1215); //opposite of driveAfterTurn
                                mTurnToCraterStep.setHeading(135);
                                mGyroAngle = 0;
                                tfod.shutdown();
                                return true;
                            }
                            else if (mGoldPositionAngle >= 15) {//right
                                mGoldPosition = 2;
                                mTurnStep.setHeading(-30);
                                mDriveAfterTurn.set(-.25f, -.25f, -1575, -1575);
                                mDriveBacktoOrigin.set(-.25f, -.25f, 1575, 1575);
                                mReturnStep.setHeading(45);
                                mGyroAngle = -45;
                                tfod.shutdown();
                                return true;
                            }
                            mOpMode.telemetry.addData("Gold Position", mGoldPosition);
                        }
                    }
                }
            }
        }

        /*if(mTimer.done()) {
            if (goldMineral != null) {
                mGoldPositionAngle = (float) (goldMineral.estimateAngleToObject(AngleUnit.DEGREES));
                mOpMode.telemetry.addData("Gold Position Angle", mGoldPositionAngle);
                mOpMode.telemetry.addData("Gold Position", mGoldPosition);
                if (mGoldPosition == 0) {
                    mTurnStep.set(1.0, -1.0, 450, -450);
                    mReturnStep.set(-1.0, 1.0, -850, 850);
                    mDriveAfterTurn.set(1.0, 1.0, 3000, 3000);
                    mTurnToCraterStep.set(1.0f, -1.0f, 2000, -2000);
                }
                if (mGoldPosition == 1) {
                    mTurnStep.set(0.0, 0.0, 0, 0);
                    mReturnStep.set(0.0, 0.0, 0, 0);
                    mDriveAfterTurn.set(1.0, 1.0, 2100, 2100);
                    mTurnToCraterStep.set(1.0f, -1.0f, 1600, -1600);
                }
                if (mGoldPosition == 2) {
                    mTurnStep.set(-1.0, 1.0, -500, 500);
                    mDriveAfterTurn.set(1.0, 1.0, 3100, 3100);
                    mReturnStep.set(1.0, -1.0, 850, -850);
                    mTurnToCraterStep.set(1.0f, -1.0f, 1300, -1300);
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }
            return true;
        }*/

        return false;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = mOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", mOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
