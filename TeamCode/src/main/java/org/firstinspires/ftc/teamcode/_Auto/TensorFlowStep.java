package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

import java.util.List;

public class TensorFlowStep extends AutoLib.Step {
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
    AutoLib.TurnByEncoderStep mTurnStep;
    AutoLib.TurnByEncoderStep mReturnStep;
    AutoLib.TurnByEncoderStep mTurnToCraterStep;
    AutoLib.TurnByEncoderStep mDriveAfterTurn;
    RoverRuckusHardware mRobot;
    int mGoldPosition;

    public TensorFlowStep(OpMode opMode, AutoLib.TurnByEncoderStep turnStep, RoverRuckusHardware robot, AutoLib.TurnByEncoderStep returnStep, AutoLib.TurnByEncoderStep turnToCraterStep, AutoLib.TurnByEncoderStep driveAfterTurn) {
        mOpMode = opMode;
        mTurnStep = turnStep;
        mReturnStep = returnStep;
        mDriveAfterTurn = driveAfterTurn;
        mTurnToCraterStep = turnToCraterStep;
        mDriveAfterTurn = driveAfterTurn;
        mRobot = robot;
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

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                mOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            mOpMode.telemetry.addData("Gold Mineral Position", "Left");
                            mGoldPosition = 0;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            mOpMode.telemetry.addData("Gold Mineral Position", "Right");
                            mGoldPosition = 2;
                        } else {
                            mOpMode.telemetry.addData("Gold Mineral Position", "Center");
                            mGoldPosition = 1;
                        }
                    }
                }
            }
        }

        if(mTimer.done()) {
            mOpMode.telemetry.addData("Gold Position Number", mGoldPosition);
            if(mGoldPosition == 0) {
                mTurnStep.set(1.0, -1.0, 450, -450);
                mReturnStep.set(-1.0, 1.0, -850, 850);
                mDriveAfterTurn.set(1.0, 1.0, 3000, 3000);
                mTurnToCraterStep.set(1.0f, -1.0f, 2000, -2000);
            }
            if(mGoldPosition == 1) {
                mTurnStep.set(0.0, 0.0, 0, 0);
                mReturnStep.set(0.0, 0.0, 0, 0);
                mDriveAfterTurn.set(1.0, 1.0, 2100, 2100);
                mTurnToCraterStep.set(1.0f, -1.0f, 1600, -1600);
            }
            if(mGoldPosition == 2) {
                mTurnStep.set(-1.0, 1.0, -500, 500);
                mDriveAfterTurn.set(1.0, 1.0, 3100, 3100);
                mReturnStep.set(1.0, -1.0, 850, -850);
                mTurnToCraterStep.set(1.0f, -1.0f, 1300, -1300);
            }

            if (tfod != null) {
                tfod.shutdown();
            }
            return true;
        }

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
