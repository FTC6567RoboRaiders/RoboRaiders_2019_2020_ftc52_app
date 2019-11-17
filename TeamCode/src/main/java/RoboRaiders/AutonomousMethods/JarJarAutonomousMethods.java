package RoboRaiders.AutonomousMethods;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import RoboRaiders.JarJarsAutonomous.BrightnessDetectionAuto;
import RoboRaiders.Robot.JarJarBot;
import RoboRaiders.hubbot.BrightnessDetection;

import static org.opencv.core.CvType.CV_8UC1;

public abstract class JarJarAutonomousMethods extends LinearOpMode {

    public double motor_power;
    public double degreesToTurn;
    public double currentHeading;
    public double finalHeading;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;


    public void encodersMove(JarJarBot robot, double distance, double power, String direction) { //sets the parameters

        robot.resetEncoders(); //resets encoders
        robot.runWithEncoders(); //sets the mode back to run with encoder

        final double v = robot.calculateCOUNTS(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("forward")) { //if the desired direction is forward

            robot.setDriveMotorPower(power, power, power, power); //start driving forward

            while ((double)robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                //telemetry.addData("COUNTS", COUNTS);
                //telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                //telemetry.update();
            }

            robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        } else if (direction.equals("backward")) { //if the desired direction is backward

            robot.setDriveMotorPower(-power, -power, -power, -power); //start driving backward

            while ((double)robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }
            robot.setDriveMotorPower(0,0,0,0);
        }
    }

    public void encodersMoveStrafe(JarJarBot robot, double distance, double power, String direction) {
        robot.resetEncoders();
        robot.runWithEncoders();

        final double v = robot.calculateCOUNTS(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("left")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, -power, power); //start strafing right

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        } else if (direction.equals("right")) { //if the desired direction is left

            robot.setDriveMotorPower(-power, power, power, -power); //start strafing left

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        }

        robot.runWithoutEncoders(); //sets the mode back to run without encoder
    }

    public void runIntake(JarJarBot robot, double power) {
        double startIntakeTime = System.currentTimeMillis();
        robot.setInakePower(power);
        /*while (opModeIsActive() && System.currentTimeMillis() - startIntakeTime < 4000) {
        }

        robot.setInakePower(0);*/

    }

    public void intakeArmAuto (JarJarBot robot, double position) {
      robot.intakeArmAuto(position);
    }


    public void collectStone(JarJarBot robot){
        double startIntakeTime = System.currentTimeMillis();
        runIntake(robot, -.6);
        encodersMove(robot, 28, .15, "forward");
        while (opModeIsActive() && System.currentTimeMillis() - startIntakeTime < 4000){
        }
        runIntake(robot, 0);
    }

    public void robotSleep(int timeToSleep) {
        try {
            Thread.sleep(timeToSleep);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void imuTurn(JarJarBot robot, float degreesToTurn, double power, String direction) { //gets hardware from
        //Robot and defines degrees as a
        //float, power as a double, and direction as a string
        //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addLine().addData("degreesToTurn",String.valueOf(degreesToTurn));
        currentHeading = robot.getIntegratedZAxis();
        //finalHeading = currentHeading + degreesToTurn;
        //telemetry.update();

        // robot.getHeading(); returns the current heading of the IMU

        if (direction.equals("left")) { //if the desired direction is right
            finalHeading = currentHeading - degreesToTurn;
            robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn right
            while(opModeIsActive() && robot.getIntegratedZAxis() > finalHeading) {
                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                //telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                //telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                //telemetry.update();

            }
        }
        else { //if the desired direction is left
            finalHeading = currentHeading + degreesToTurn;
            robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn left
            while(opModeIsActive() && robot.getIntegratedZAxis() < finalHeading) {
                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                //telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                //telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                //telemetry.update();
            }
        }
    }

    public void stoneDetection(JarJarBot robot, int pattern){
            OpenCvCamera phone_camera;
            RoboRaiders.JarJarsAutonomous.BrightnessDetectionAuto.SamplePipeline stone_pipeline;
            public int pattern = 999;
            public void runOpMode() throws InterruptedException{
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

                phone_camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                phone_camera.openCameraDevice();

                stone_pipeline = new RoboRaiders.JarJarsAutonomous.BrightnessDetectionAuto.SamplePipeline();
                phone_camera.setPipeline(stone_pipeline);

                phone_camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                waitForStart();

                while (opModeIsActive() && pattern == 999) {
                    //super.updateTelemetry(telemetry);
                    //telemetry.addData("FRAME", phone_camera.getFrameCount());
                    //telemetry.addData("FPS", String.format("%.2f", phone_camera.getFps()));
                    //telemetry.addData("TFT MS", phone_camera.getTotalFrameTimeMs());
                    //telemetry.addData("PT MS", phone_camera.getPipelineTimeMs());
                    //telemetry.addData("OT MS", phone_camera.getOverheadTimeMs());
                    //telemetry.addData("MAX FPS", phone_camera.getCurrentPipelineMaxFps());
                    //telemetry.addData("LEFT RECT", stone_pipeline.left_hue + " " + stone_pipeline.left_br);
                    //telemetry.addData("RIGHT RECT", stone_pipeline.right_hue + " " + stone_pipeline.right_br);
                    //telemetry.addData("PATTERN", pattern);
                    //telemetry.update();
                }

                phone_camera.stopStreaming();

                telemetry.addData("PATTERN", pattern);
                telemetry.update();

                Thread.sleep(2000);
            }

            class SamplePipeline extends OpenCvPipeline {
                int left_hue;
                int right_hue;

                int left_br;
                int right_br;


                public Mat processFrame(Mat input, int pattern) {

                    input.convertTo(input, CV_8UC1, 1, 10);

                    int[] left_rect = {
                            //the second number here possibly represents the amount of sections you are splitting the screen into??   32
                            (int) (input.cols() * (9f / 32f)), //the first number goes from left to right increasing, controls x axis
                            (int) (input.rows() * (5f / 32f)), //the first number here controls the y axis
                            (int) (input.cols() * (15f / 32f)),
                            (int) (input.rows() * (7f / 32f))
                    };

                    int[] right_rect = {
                            (int) (input.cols() * (17f / 32f)),
                            (int) (input.rows() * (5f / 32f)),
                            (int) (input.cols() * (23f / 32f)),
                            (int) (input.rows() * (7f / 32f))
                    };

                    Imgproc.rectangle(
                            input,
                            new Point(
                                    left_rect[0],
                                    left_rect[1]),

                            new Point(
                                    left_rect[2],
                                    left_rect[3]),
                            new Scalar(0, 255, 0), 1);

                    Imgproc.rectangle(
                            input,
                            new Point(
                                    right_rect[0],
                                    right_rect[1]),

                            new Point(
                                    right_rect[2],
                                    right_rect[3]),
                            new Scalar(0, 0, 255), 1);

                    Mat left_block = input.submat(left_rect[1], left_rect[3], left_rect[0], left_rect[2]);
                    Mat right_block = input.submat(right_rect[1], right_rect[3], right_rect[0], right_rect[2]);


                    Scalar left_mean = Core.mean(left_block);


                    Scalar right_mean = Core.mean(right_block);

                    left_hue = get_hue((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
                    right_hue = get_hue((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);
                    left_br = get_brightness((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
                    right_br = get_brightness((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);

                    if (left_br > 100 && right_br > 100) pattern = 1;
                        //skystone is not in frame
                        //above 100 is normal, bellow 100 is skystone
                    else if (left_br > 100 && right_br < 100) pattern = 2;
                        //skystone is on right
                    else if (left_br < 100 && right_br > 100) pattern = 3;
                        //skystone is on left
                    else if (left_br < 100 && right_br < 100) {
                        if (left_br > right_br) {
                            pattern = 1;
                        } else if (left_br < right_br) {
                            pattern = 2;
                        } else {
                            pattern = 3;
                        }
                    }
                    telemetry.addData("position", pattern);
                    telemetry.update();
                    sleep(100);

                    return input;
                }
            }


            private int get_hue(int red, int green, int blue) {

                float min = Math.min(Math.min(red, green), blue);
                float max = Math.max(Math.max(red, green), blue);

                if (min == max) {
                    return 0;
                }

                float hue = 0f;
                if (max == red) {
                    hue = (green - blue) / (max - min);

                } else if (max == green) {
                    hue = 2f + (blue - red) / (max - min);

                } else {
                    hue = 4f + (red - green) / (max - min);
                }

                hue = hue * 60;
                if (hue < 0) hue = hue + 360;

                return Math.round(hue);
            }

            private int get_brightness(int red, int green, int blue) {
                return (int) (((double) (red + green + blue)) / 3);
            }

    }
}
