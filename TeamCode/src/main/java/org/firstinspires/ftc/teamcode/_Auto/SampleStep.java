package org.firstinspires.ftc.teamcode._Auto;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;
import org.firstinspires.ftc.teamcode._Libs.hardware.RoverRuckusHardware;

public class SampleStep extends AutoLib.Step {

    VuforiaLib_RoverRuckus mVLib;
    OpMode opMode;
    int cubePositionCount[], cubePosition;
    AutoLib.Timer mTimer;
    boolean save;
    AutoLib.TurnByEncoderStep mTurnStep;
    AutoLib.TurnByEncoderStep mReturnStep;
    AutoLib.TurnByEncoderStep mTurnToCraterStep;
    AutoLib.TurnByEncoderStep mDriveAfterTurn;
    RoverRuckusHardware mRobot;

    public SampleStep(VuforiaLib_RoverRuckus mVLib, OpMode opMode) {
        this.mVLib = mVLib;
        this.opMode = opMode;
        this.cubePositionCount = new int[3];
        this.save = false;
    }

    public SampleStep(VuforiaLib_RoverRuckus mVLib, OpMode opMode, AutoLib.TurnByEncoderStep turnStep, RoverRuckusHardware robot, AutoLib.TurnByEncoderStep returnStep, AutoLib.TurnByEncoderStep turnToCraterStep, AutoLib.TurnByEncoderStep driveAfterTurn){
        this.mVLib = mVLib;
        this.opMode = opMode;
        this.cubePositionCount = new int[3];
        this.save = true;
        mTimer = new AutoLib.Timer(1.0);

        mTurnStep = turnStep;
        mReturnStep = returnStep;
        mDriveAfterTurn = driveAfterTurn;
        mTurnToCraterStep = turnToCraterStep;
        mDriveAfterTurn = driveAfterTurn;
        mRobot = robot;
    }

    @Override
    public boolean loop() {
        Bitmap bmIn = mVLib.getBitmap(4);
        int bmArray[] = new int[bmIn.getWidth()*bmIn.getWidth()];
        int bmOutArray[] = new int[bmIn.getWidth()*bmIn.getHeight()];
        int cubeX = 0;

        if(firstLoopCall() && mTimer != null) {
            mTimer.start();
        }

        if (bmIn != null) {
            // create the output bitmap we'll display on the RC phone screen
            bmIn.getPixels(bmArray, 0, bmIn.getWidth(), 0, 0, bmIn.getWidth(), bmIn.getHeight());
            convertToSimpleColorRaster(bmArray, bmOutArray, bmIn.getHeight(), bmIn.getWidth(), bmIn.getWidth());
            cubeX = findCube(bmOutArray, bmIn.getHeight(), bmIn.getWidth());
            if (cubeX < bmIn.getWidth() / 3) {
                cubePositionCount[0]++;
            } else if (cubeX > bmIn.getWidth() * 2 / 3) {
                cubePositionCount[2]++;
            } else {
                cubePositionCount[1]++;
            }

            if(cubePositionCount[0] > cubePositionCount[1] && cubePositionCount[0] > cubePositionCount[2]) {
                cubePosition = 1;
            }
            else if(cubePositionCount[1] > cubePositionCount[0] && cubePositionCount[1] > cubePositionCount[2]) {
                cubePosition = 2;
            }
            else {
                cubePosition = 3;
            }

            opMode.telemetry.addData("Cube X", cubePosition);

            if(mTimer != null && mTimer.done() && save) {
                if(cubePosition == 3) {
                    mTurnStep.set(1.0, -1.0, 450, -450);
                    mReturnStep.set(-1.0, 1.0, -850, 850);
                    mDriveAfterTurn.set(1.0, 1.0, 3000, 3000);
                    mTurnToCraterStep.set(1.0f, -1.0f, 2000, -2000);
                }
                if(cubePosition == 2) {
                    mTurnStep.set(0.0, 0.0, 0, 0);
                    mReturnStep.set(0.0, 0.0, 0, 0);
                    mDriveAfterTurn.set(1.0, 1.0, 2000, 2000);
                    mTurnToCraterStep.set(1.0f, -1.0f, 1600, -1600);
                }
                if(cubePosition == 1) {
                    mTurnStep.set(-1.0, 1.0, -500, 500);
                    mDriveAfterTurn.set(1.0, 1.0, 3100, 3100);
                    mReturnStep.set(1.0, -1.0, 850, -850);
                    mTurnToCraterStep.set(1.0f, -1.0f, 1300, -1300);
                }
                return true;
            }
        }
        return super.loop();
    }

    public static void convertToSimpleColorRaster(int rgb[], int[] simple, int nrows, int ncols, int frameWidth) {
        /*
         *d
         *Serves color raster encoded in 1D of values 0-5 with
         * 0 = RED
         * 1 = GREEN
         * 2 = BLUE
         * 3 = WHITE
         * 4 = GREY
         * 5 = BLACK
         * 6 = YELLOW
         */
        for(int r = 0; r < nrows; r++){
            for(int c = 0; c < frameWidth; c++){
                int red = (rgb[ncols*r+c]&0xFF0000)>>16;
                int green = (rgb[ncols*r+c]&0xFF00)>>8;
                int blue = (rgb[ncols*r+c]&0xFF);
                //int B = (((int)bayer[(r*ncols*2 + c)*2 + 1+2*ncols-ncols*2*getBit(tile,1)-getBit(tile,0)])&0xFF);			//Bottom right (blue)

                double Y = red *  .299000 + green *  .587000 + blue *  .114000;
                double U  = red * -.168736 + green * -.331264 + blue *  .500000 + 128;
                double V = red *  .500000 + green * -.418688 + blue * -.081312 + 128;
                if(r < nrows / 2){
                    simple[ncols*r+c] = 0xFF000000;
                }else if(red > 2.25*blue && green > 1.75*blue && red > 80){
                    simple[r*ncols+c] = 0xFFFFFF00;
                }else {
                    simple[ncols * r + c] = (int) Y | (int) Y << 8 | (int) Y << 16 | 0xFF000000;
                    //simple[ncols*r+c] = 0xFF000000;
                }
                /*
                red =(int)(  1.4075 * (V - 128));
                green = (int)(0- 0.3455 * (U - 128) - (0.7169 * (V - 128)));
                blue = (int)(1.7790 * (U - 128));
                */
                //If one of the colors has a value 50 greater than both other colors
                //it assigns that pixel to that color
                /*
                if(red > green+50 && red > blue+50){
                    simple[r*ncols+c] = 0xFFFF0000;
                } else if(green > red+40 && green > blue-10){
                    simple[r*ncols+c] = 0xFF00FF00;
                } else if(blue > red+50 && blue > green+50){
                    simple[r*ncols+c] = 0xFF0000FF;
                }else if(red<green+25&&green<red+25&&(red>blue+70)){
                    simple[r*ncols+c] = 0xFFFFFF00;
                }
                //Otherwise it sees if one of the colors has a value above 170 for white
                // if not, 85 for grey and below 85 for black
                else if(Y>140){
                    simple[r*ncols+c] = 0xFFFFFFFF;
                } else if(Y>85){
                    simple[r*ncols+c] = 0xFF808080; //0x808080
                } else {
                    simple[r * ncols + c] = 0xFF000000;
                }
                */
            }
        }
    }

    public int findCube(int input[], int nrows, int ncols){
        int averageX[] = new int[nrows];
        int total = 0;
        int counted = 0;
        for(int r = 0; r < nrows; r++ ){
            int averageRowX = 0;
            int numYellow = 0;
            for(int c = 0; c < ncols; c++){
                if(input[r*ncols+c] == 0xFFFFFF00){
                    averageRowX += c;
                    numYellow++;
                }
            }
            if(numYellow > 0){
                averageX[r] = averageRowX/numYellow;
            }
        }
        for(int i = 0; i < averageX.length; i++){
            if(averageX[i] > 0){
                total += averageX[i];
                counted++;
            }
        }
        if(counted > 0) {
            return total / counted;
        } else{
            return 0;
        }
    }


}

