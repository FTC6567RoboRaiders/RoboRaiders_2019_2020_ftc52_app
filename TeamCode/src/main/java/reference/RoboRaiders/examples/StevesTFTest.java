/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package RoboRaiders.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import RoboRaiders.Logger.LoggerOId;
import RoboRaiders.Robot.RobotTelemetryDisplay;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Steeeves Tensorflow Test", group = "Test")
@Disabled
public class StevesTFTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AedUDNP/////AAAAGXH2ZpUID0KanSX9ZSR37LKFSFokxIqmy/g0BNepdA9EepixxnO00qygLnMJq3Fg9gZxnkUJaKgk14/UjhxPWVQIs90ZXJLc21NvQvOeZ3dOogagVP8yFnFQs2xCijGmC/CE30ojlAnbhAhqz1y4tZPW2QkK5Qt0xCakTTSAw3KPQX2mZxX+qMxI2ljrN0eaxaKVnKnAUl8x3naF1mez7f9c8Xdi1O5auL0ePdG6bJhWjEO1YwpSd8WkSzNDEkmw20zpQ7zaOOPw5MeUQUr9vAS0fef0GnLjlS1gb67ajUDlEcbbbIeSrLW/oyRGTil8ueQC2SWafdspSWL3SJNaQKWydies23BxJxM/FoLuYYjx";

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
    private int goldPostion = -1;

    private long  startTimeInMillis  = 0;
    private long  endTimeInMillis    = 0;
    private int   numberOfRecognitions = 0;
    private int   iterations           = 0;

    private double elapsedTimeInSecs = 0.0;
    private long  whileStartTimeInMillis = 0;
    private long  whileEndTimeInMillis   = 0;
    private double whileElapsedTimeInSecs = 0.0;

    private List<Recognition> updatedRecognitions;
    private RobotTelemetryDisplay rtd;
    private LoggerOId myLogger;


    @Override
    public void runOpMode() {

        rtd = new RobotTelemetryDisplay(this, "HubBot");
        myLogger = new LoggerOId("SMKSMK");

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        startTimeInMillis = System.currentTimeMillis();
        initVuforia();
        endTimeInMillis   = System.currentTimeMillis();
        elapsedTimeInSecs = (double)(endTimeInMillis - startTimeInMillis) / 1000.0;

        rtd.displayRobotTelemetry("Elapsed Time");
        rtd.displayRobotTelemetry("Vuforia",String.valueOf(elapsedTimeInSecs));
        myLogger.Debug("Elapsed Time");
        myLogger.Debug("Vuforia: ",elapsedTimeInSecs);


        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            startTimeInMillis = System.currentTimeMillis();
            initTfod();
            endTimeInMillis   = System.currentTimeMillis();
            elapsedTimeInSecs = (double)(endTimeInMillis - startTimeInMillis) / 1000.0;
            rtd.displayRobotTelemetry("TFObjectDetector",String.valueOf(elapsedTimeInSecs));
            myLogger.Debug("TFObjectDector: ",elapsedTimeInSecs);

        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        //rtd.displayRobotTelemetry("Press Start","To Begin");

        if (tfod != null) {
            startTimeInMillis = System.currentTimeMillis();
            tfod.activate();
            endTimeInMillis   = System.currentTimeMillis();
            elapsedTimeInSecs = (double)(endTimeInMillis - startTimeInMillis) / 1000.0;
            rtd.displayRobotTelemetry("TFObjectDetector Activate",String.valueOf(elapsedTimeInSecs));
            myLogger.Debug("TFObjectDetector Active: ",elapsedTimeInSecs);
        }
        waitForStart();
        myLogger.Debug("Robot Started");

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */


            whileStartTimeInMillis = System.currentTimeMillis();
            CameraDevice.getInstance().setFlashTorchMode(true);
            while (opModeIsActive()) { // && (updatedRecognitions == null || numberOfRecognitions < 3)) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.

              //      startTimeInMillis = System.currentTimeMillis();
                    updatedRecognitions = tfod.getUpdatedRecognitions();
              //      endTimeInMillis = System.currentTimeMillis();
              //      elapsedTimeInSecs = (double)(endTimeInMillis - startTimeInMillis) / 1000.0;
              //      rtd.displayRobotTelemetry("TFObjectDetector Recognitions", String.valueOf(elapsedTimeInSecs));
                    if (updatedRecognitions == null) {
                        numberOfRecognitions = 0;
                    }
                    else {
                        numberOfRecognitions = updatedRecognitions.size();
                    }
                    if (numberOfRecognitions > 0) {
                        rtd.displayRobotTelemetry("Number of Objects", String.valueOf(numberOfRecognitions));
                        for (Recognition recognition : updatedRecognitions) {
                            rtd.displayRobotTelemetry("confidence", String.valueOf(recognition.getConfidence()));
                        }

                    }
               //     myLogger.Debug("TFObjectDetector Recognitions: ",elapsedTimeInSecs);
               //     myLogger.Debug("Number of Objects: ",numberOfRecognitions);
                }

              //  iterations++;
             //   rtd.displayRobotTelemetry("interation #", String.valueOf(iterations));
             //   myLogger.Debug("iteration #: ",iterations);
            }
            whileEndTimeInMillis = System.currentTimeMillis();
            whileElapsedTimeInSecs = (double)(whileEndTimeInMillis - whileStartTimeInMillis) / 1000.0;
            rtd.displayRobotTelemetry("While Loop", String.valueOf(whileElapsedTimeInSecs));
            myLogger.Debug("While Loop: ",whileElapsedTimeInSecs);

/*

                    if (updatedRecognitions != null) {

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
                            telemetry.addData("Gold Mineral Position", "Left");
                            goldPostion = 1;//indicate gold mineral is in the left position
                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            goldPostion = 3;//indicate gold mineral is in the right position
                          } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            goldPostion = 2;//indicate gold mineral is in the center position
                          }
                        }

                      }
                        if (updatedRecognitions.size() == 2){
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
                            //difference starts here
                            if (silverMineral2X != -1){
                                goldPostion = 1;
                            }
                            else {
                                if(goldMineralX < silverMineral1X){
                                    goldPostion = 2;
                                }
                                else {
                                    goldPostion = 3;
                                }
                            }
                        }
                      telemetry.update();
                    }
                }
            }
*/
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        //return goldPostion;
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(){
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
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.80;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
