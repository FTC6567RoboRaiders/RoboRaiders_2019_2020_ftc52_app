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
import RoboRaiders.JarJarsAutonomous.RoboRaidersPipeline;
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
        while (opModeIsActive() && System.currentTimeMillis() - startIntakeTime < 3000) {
        }

        robot.setInakePower(0);

    }

    public void intakeArmAuto (JarJarBot robot, double position) {
      robot.intakeArmAuto(position);
    }

    public void stoneSampleServo (JarJarBot robot) {
        intakeArmAuto(robot, 0.0);
        robotSleep(1000);
        encodersMove(robot, 15, .3, "backward");
        robotSleep(1000);
        encodersMove(robot, 2, .3, "forward");
        robotSleep(1000);
        runIntake(robot, -.5);
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

    public int stoneDetection(){
            OpenCvCamera phone_camera;
            RoboRaidersPipeline stone_pipeline;
            int pattern = 999;
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            phone_camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phone_camera.openCameraDevice();
            stone_pipeline = new RoboRaidersPipeline(pattern);
            phone_camera.setPipeline(stone_pipeline);

            phone_camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            while (opModeIsActive() && stone_pipeline.getPattern() == 999) {
                telemetry.addData("FRAME", phone_camera.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", phone_camera.getFps()));
                telemetry.addData("TFT MS", phone_camera.getTotalFrameTimeMs());
                telemetry.addData("PT MS", phone_camera.getPipelineTimeMs());
                telemetry.addData("OT MS", phone_camera.getOverheadTimeMs());
                telemetry.addData("MAX FPS", phone_camera.getCurrentPipelineMaxFps());
            }
            phone_camera.stopStreaming();
            telemetry.addData("PATTERN", pattern);
            telemetry.update();
            return stone_pipeline.getPattern();

    }

    public void stoneSampling(JarJarBot robot){ //NOTE THE PATTERNS MAY OR MAY NOT BE SCREWED UP!!!
        int stoneLocation = stoneDetection();

        switch (stoneLocation){
            case 1: //stone is on leftmost (not if the frame)
                leftStone(robot);
                break;
            case 2: //stone is on the left (middle)
                middleStone(robot);
                break;
            case 3: //stone is on the right
                rightStone(robot);
                break;
            case 999:
                middleStone(robot);
                break;
        }
    }


    public void leftStone(JarJarBot robot){
        encodersMove(robot, 20, .4, "forward");
        robotSleep(1000);
        encodersMoveStrafe(robot, 3, .1, "right");
        robotSleep(1000);
        stoneSampleServo(robot);

    }

    public void middleStone(JarJarBot robot){
        encodersMove(robot, 3, .4, "forward");

    }

    public void rightStone(JarJarBot robot){
        encodersMove(robot, 3, .4, "forward");

    }
}
