package RoboRaiders.AutonomousMethods;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import RoboRaiders.Autonomous.RoboRaidersPipeline;
import RoboRaiders.Autonomous.RoboRaidersPipelineWebcam;
import RoboRaiders.Logger.Logger;
import RoboRaiders.Robot.Robot;
import RoboRaiders.hubbot.BrightnessDetectionWebcam;

public abstract class RRAutonomousMethods extends LinearOpMode {

    public double motor_power;
    public double degreesToTurn;
    public double currentHeading;
    public double finalHeading;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;


    public void liftMotorRTPDriveWithStone(Robot robot) {
        robot.resetLiftEncoder();
        robot.runLiftWithEncoderRTP();
        robot.setLiftMotorTargetPosition(18); //18 encoders is equal to 1/8 inch up
        robot.setLiftMotorPower(.3);
    }

    public void liftMotorRTPDriveWithoutStone(Robot robot, double liftPower, int position) {
        robot.resetLiftEncoder();
        robot.runLiftWithEncoderRTP();
        robot.setLiftMotorTargetPosition(-18); //-18 encoders is equal to 1/8 inch down
        robot.setLiftMotorPower(.3);
    }

    public void encodersMoveRTP(Robot robot, double distance, double power, String direction){
        robot.resetEncoders();
        robot.runWithEncodersSTP();

        final double v = robot.driveTrainCalculateCounts(distance);
        double COUNTS = v;

        if (direction.equals("forward")) {
            robot.setDTMotorTargetPosition((int)COUNTS);
            robot.setDriveMotorPower(power, power, power, power);

            while ((double)robot.getSortedEncoderCount() < COUNTS && opModeIsActive()){
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }
            robot.setDriveMotorPower(0, 0, 0, 0);
        }

        if (direction.equals("backward")) {
            robot.setDTMotorTargetPosition(-(int)COUNTS);
            robot.setDriveMotorPower(-power, -power, -power, -power);

            while (-(double)robot.getSortedEncoderCount() > -COUNTS && opModeIsActive()){
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }
            robot.setDriveMotorPower(0, 0, 0, 0);
        }
        robot.resetEncoders();
        robot.runWithoutEncoders();
    }

    public void encodersMove(Robot robot, double distance, double power, String direction) { //sets the parameters

        robot.resetEncoders(); //resets encoders
        robot.runWithEncoders(); //sets the mode back to run with encoder

        final double v = robot.driveTrainCalculateCounts(distance);
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

    public void encodersMoveStrafe(Robot robot, double distance, double power, String direction) {
        robot.resetEncoders();
        robot.runWithEncoders();

        final double v = robot.driveTrainCalculateCounts(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, -power, power); //start strafing right

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        } else if (direction.equals("left")) { //if the desired direction is left

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

    public void runIntake(Robot robot, double power) {
        robot.setIntakePower(power);
    }

    /*public void intakeArmAuto (Robot robot, double position) {
      robot.intakeArmAuto(position);
    }*/

    public void stoneSampleServo (Robot robot) {
       // intakeArmAuto(robot, 0.0);
        robotSleep(1000);
        encodersMove(robot, 15, .3, "backward");
        robotSleep(1000);
        encodersMove(robot, 2, .3, "forward");
        robotSleep(1000);
        //runIntake(robot, -.5);
    }


    public void collectStone(Robot robot){
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

    public void imuTurn(Robot robot, float degreesToTurn, double power, String direction) { //gets hardware from
        //Robot and defines degrees as a
        //float, power as a double, and direction as a string
        //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addLine().addData("degreesToTurn",String.valueOf(degreesToTurn));
        currentHeading = robot.getIntegratedZAxis();
        //finalHeading = currentHeading + degreesToTurn;
        //telemetry.update();

        // robot.getHeading(); returns the current heading of the IMU

        if (direction.equals("left")) { //if the desired direction is left
            finalHeading = currentHeading + degreesToTurn;
            robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn right
            while(opModeIsActive() && robot.getIntegratedZAxis() < finalHeading) {
                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                telemetry.addLine().addData("finalheading",String.valueOf(finalHeading));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                telemetry.update();

            }
        }
        else { //if the desired direction is right
            finalHeading = currentHeading - degreesToTurn;
            robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
            while(opModeIsActive() && robot.getIntegratedZAxis() > finalHeading) {
                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                telemetry.addLine().addData("finalheading",String.valueOf(finalHeading));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                telemetry.update();
            }
        }
        robot.setDriveMotorPower(0,0,0,0);
    }

    /**
     * turning with PID
     * @param robot
     * @param rrPID
     * @param degreesToTurn
     * @param direction
     */
    public void imuTurnPID(RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID rrPID,
                           Robot robot,
                           float degreesToTurn, String direction) {

        double power = 0.0;
        int loopcount = 0;


        // Normally the motor powers for the left side are set to reverse (this allows the motors
        // to turn in the same direction.  For turning however, set the motors on the left side of
        // the robot to turn forward, which will turn the left motors in the opposite direction
        // of the right motors, thus turning the robot.
        robot.motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.motorBackLeft.setDirection(DcMotor.Direction.FORWARD);

        Logger L = new Logger("imuTurnPID");
        rrPID.initialize();
        telemetry.addLine().addData("in", "imuTurnPID");

        currentHeading = 0.0;
        currentHeading = robot.getIntegratedZAxis();


        L.Debug("Start");
        L.Debug("currentHeading: ", currentHeading);
        L.Debug("degreesToTurn", degreesToTurn);

        // robot.getHeading(); returns the current heading of the IMU

        // When turning and reading the IMU...clockwise angles generally decrease, that is they
        // get smaller.  Whereas, counter clockwise angles generally increase, that is they
        // get larger.  Thus turning right requires the number of degrees to turn to be decremented
        // from the current heading.  For turning left, the number of degrees to turn is incremented
        // to the current heading.

        if (direction.equals("right")) { //if the desired direction is right
            finalHeading = currentHeading - degreesToTurn;
            telemetry.addLine().addData("currentHeading", currentHeading);
            telemetry.addLine().addData("finalHeading", finalHeading);

            L.Debug("Turning Right");
            L.Debug("finalHeading: ",finalHeading);

            // power = rrPID.CalculatePIDPowers(finalHeading,currentHeading);
            telemetry.addLine().addData("power", power);

            L.Debug("Calculated PID Power (power): ",power);

            //  robot.setDriveMotorPower(power, power, power, power); //the robot will turn right


            // The robot will turn within plus or minus 3.5 degrees and needs to complete the turn
            // in 20 iterations or less.  During testing it was found that the robot would still be
            // applying small amounts of power in an attempt to get the robot to turn the last
            // few degrees, however, the small amounts of power wasn't quite enough to over come the
            // effects of friction on the robot

            while((opModeIsActive() && (loopcount < 20 &&
                    !(currentHeading < finalHeading + 3.5 && currentHeading > finalHeading - 3.5)))){
                //&& Math.abs(power) > 0.1) {
                currentHeading = robot.getIntegratedZAxis();
                power = rrPID.CalculatePIDPowers(finalHeading,currentHeading) * 0.75;

                loopcount++;

                L.Debug("In While Loop");
                L.Debug("finalHeading: ",finalHeading);
                L.Debug("currentHeading: ",currentHeading);
                L.Debug("Remaining Degrees: ",finalHeading - currentHeading);
                L.Debug("Calculated PID Power (power): ",power);
                L.Debug("loopcount", loopcount);


                robot.setDriveMotorPower(power, power, power, power);

                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("right", "right");
                telemetry.addLine().addData("power", String.valueOf(power));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.getIntegratedZAxis()));
                telemetry.addLine().addData("finalHeading",String.valueOf(finalHeading));
                telemetry.addLine().addData("difference",String.valueOf(finalHeading - robot.getIntegratedZAxis()));
                telemetry.update();

            }
            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
            L.Debug("Out of While Loop");
            L.Debug("finalHeading: ",finalHeading);
            L.Debug("currentHeading: ",robot.getIntegratedZAxis());
            L.Debug("Remaining Degrees: ",finalHeading - robot.getIntegratedZAxis());



        }
        else { //if the desired direction is left
            finalHeading = currentHeading + degreesToTurn;

            L.Debug("Turning Left");
            L.Debug("finalHeading: ",finalHeading);


            // The robot will turn within plus or minus 3.5 degrees and needs to complete the turn
            // in 20 iterations or less.  During testing it was found that the robot would still be
            // applying small amounts of power in an attempt to get the robot to turn the last
            // few degrees, however, the small amounts of power wasn't quite enough to over come the
            // effects of friction on the robot

            while((opModeIsActive() && (loopcount < 20 &&
                    !(currentHeading > finalHeading - 3.5 && currentHeading < finalHeading + 3.5)))){
                //&& Math.abs(power) > 0.1) {
                currentHeading = robot.getIntegratedZAxis();
                power = rrPID.CalculatePIDPowers(finalHeading,currentHeading) * 0.75;

                loopcount++;

                L.Debug("In While Loop");
                L.Debug("finalHeading: ",finalHeading);
                L.Debug("currentHeading: ",currentHeading);
                L.Debug("Remaining Degrees: ",finalHeading - currentHeading);
                L.Debug("Calculated PID Power (power): ",power);
                L.Debug("loopcount", loopcount);


                robot.setDriveMotorPower(power, power, power, power);


                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("left", "left");
                //telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                telemetry.addLine().addData("finalHeading",String.valueOf(finalHeading));
                telemetry.addLine().addData("difference",String.valueOf(finalHeading - robot.getIntegratedZAxis()));
                telemetry.update();
            }
        }


        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot


        // Reverse the motor direction on the left motors so that all motors spin in the same
        // direction
        robot.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);


        L.Debug("End");
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
                telemetry.addData("PATTERN", stone_pipeline.getPattern());
                telemetry.update();
            }
            phone_camera.stopStreaming();
            telemetry.addData("PATTERN", stone_pipeline.getPattern());
            telemetry.update();
            return stone_pipeline.getPattern();

    }



    public int stoneDetectionWebcam(float[] leftRec, float[] rightRec){

        OpenCvCamera webcam;
        RoboRaidersPipelineWebcam stone_pipeline;
        int pattern = 999;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        stone_pipeline = new RoboRaidersPipelineWebcam(pattern, leftRec, rightRec);

        webcam.setPipeline(stone_pipeline);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        while (opModeIsActive() && stone_pipeline.getPattern() == 999) {
            telemetry.addData("FRAME", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("TFT MS", webcam.getTotalFrameTimeMs());
            telemetry.addData("PT MS", webcam.getPipelineTimeMs());
            telemetry.addData("OT MS", webcam.getOverheadTimeMs());
            telemetry.addData("MAX FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("PATTERN", stone_pipeline.getPattern());
            telemetry.update();
        }
        webcam.stopStreaming();
        telemetry.addData("PATTERN", stone_pipeline.getPattern());
        telemetry.update();
        return stone_pipeline.getPattern();

    }

    public void stoneSamplingRed(Robot robot){ //NOTE THE PATTERNS MAY OR MAY NOT BE SCREWED UP!!!
        int stoneLocation = stoneDetection();

        switch (stoneLocation){
            case 1: //stone is on leftmost (not if the frame)
                leftStoneRed(robot);
                break;
            case 3: //stone is on the left (middle)
                rightStoneRed(robot);
                break;
            case 2: //stone is on the right
                middleStoneRed(robot);
                break;
            case 999:
                middleStoneRed(robot);
                break;
        }
    }



    public void stoneSamplingWebcamRed(Robot robot){ //NOTE THE PATTERNS MAY OR MAY NOT BE SCREWED UP!!!

        float leftRec[]  = {9f, 5f, 15f, 15f};
        float rightRec[] = {9f, 15f, 15f, 25f};
        int stoneLocation = stoneDetectionWebcam(leftRec, rightRec);

        // Gets first sky stone in the quarry.  The first skystone is furtherest away from the
        // field perimeter
        switch (stoneLocation){
            case 1: //stone is on leftmost (not if the frame)
                leftStoneRed(robot);
                break;
            case 3: //stone is on the left (middle)
                rightStoneRed(robot);
                break;
            case 2: //stone is on the right
                middleStoneRed(robot);
                break;
            case 999:
                middleStoneRed(robot);
                break;
        }

        /**
         * at this point the robot has deposited the first stone on the foundation!
         * reset the robot's lift and swing arm and have it strafe back into position
         * for the return trip
         */


    }

    public void stoneSamplingWebcamBlue(Robot robot){ //NOTE THE PATTERNS MAY OR MAY NOT BE SCREWED UP!!!

        float leftRec[]  = {9f, 2f, 15f, 11f};
        float rightRec[] = {9f, 10f, 15f, 22f};
        int stoneLocation = stoneDetectionWebcam(leftRec, rightRec);


        // Gets first sky stone in the quarry.  The first skystone is furtherest away from the
        // field perimeter
        switch (stoneLocation){
            case 1: //stone is on leftmost (not if the frame)
                rightStoneBlue(robot);
                break;
            case 3: //stone is on the left (middle)
                leftStoneBlue(robot);
                break;
            case 2: //stone is on the right
                middleStoneBlue(robot);
                break;
            case 999:
                middleStoneBlue(robot);
                break;
        }

        /**
         * at this point the robot has deposited the first stone on the foundation!
         * reset the robot's lift and swing arm and have it strafe back into position
         * for the return trip
         */





        // Could add an option here that says, if we don't want to do the second skystone
        // then just go and park
        // Get second sky stone in quarry.  The second skystone is closest to the field
        // perimeter
//        switch (stoneLocation){
//            case 1: //stone is on leftmost (not if the frame)
//                left2ndSkyStone(robot);
//                break;
//            case 3: //stone is on the left (middle)
//                right2ndSkyStone(robot);
//                break;
//            case 2: //stone is on the right
//                middle2ndSkyStone(robot);
//                break;
//            case 999:
//                middle2ndSkyStone(robot);
//                break;
//        }
    }

    public void leftStoneBlue(Robot robot){
        encodersMoveRTP(robot, 18, .8, "forward");
        imuTurn(robot, 83, .2, "right");
        encodersMoveRTP(robot, 10, .6, "backward");
        encodersMoveStrafe(robot, 19, .5, "left");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, .2, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500){}
        runIntake(robot, 0.0);
        robot.setCaptureServoDown();
        robotSleep(500);
        liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 13, .8, "right");
        encodersMoveRTP(robot, 70, .8, "backward");
        encodersMoveStrafe(robot, 10, .5, "right");
        stoneOnFoundation(robot);
        resetStoneMechanism(robot);
        encodersMoveStrafe(robot, 5, 0.5, "left");
    }

    public void rightStoneBlue(Robot robot){
        encodersMove(robot, 18, .8, "forward");
        encodersMoveStrafe(robot, 25, .5, "left");
        imuTurn(robot, 20, .5, "right");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 20, .5, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 5000){}
        runIntake(robot, 0.0);
        robot.setCaptureServoDown();
        robotSleep(500);
        liftMotorRTPDriveWithStone(robot);
        encodersMoveRTP(robot, 20, .8, "backward");
        imuTurn(robot, 55, .5, "right");
        encodersMoveStrafe(robot, 20, .5, "right");
        encodersMoveRTP(robot, 59, .8, "backward");
        stoneOnFoundation(robot);
        resetStoneMechanism(robot);
        encodersMoveStrafe(robot, 5, 0.5, "left");
    }

    public void middleStoneBlue(Robot robot){
        encodersMoveRTP(robot, 18, .8, "forward");
        imuTurn(robot, 83, .2, "right");
        encodersMoveRTP(robot, 15.5, .6, "backward");
        encodersMoveStrafe(robot, 19, .5, "left");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, .2, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        robot.setCaptureServoDown();
        robotSleep(500);
        liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 13, .8, "right");
        encodersMoveRTP(robot, 67, .8, "backward");
        encodersMoveStrafe(robot, 10, .5, "right");
        stoneOnFoundation(robot);
        resetStoneMechanism(robot);
        encodersMoveStrafe(robot, 5, 0.5, "left");
    }

    public void leftStoneRed(Robot robot){
        encodersMoveRTP(robot, 18, .8, "forward");
        imuTurn(robot, 83, .2, "left");
        encodersMoveRTP(robot, 10, .6, "backward");
        encodersMoveStrafe(robot, 19, .5, "right");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, .2, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500){}
        runIntake(robot, 0.0);
        robot.setCaptureServoDown();
        robotSleep(500);
        liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 13, .8, "left");
        encodersMoveRTP(robot, 70, .8, "backward");
        encodersMoveStrafe(robot, 10, .5, "left");
        stoneOnFoundation(robot);
        resetStoneMechanism(robot);
        encodersMoveStrafe(robot, 5, 0.5, "right");
    }

    public void middleStoneRed(Robot robot){
        encodersMoveRTP(robot, 18, .8, "forward");
        imuTurn(robot, 83, .2, "left");
        encodersMoveRTP(robot, 15.5, .6, "backward");
        encodersMoveStrafe(robot, 19, .5, "right");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, .2, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        robot.setCaptureServoDown();
        robotSleep(500);
        liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 13, .8, "left");
        encodersMoveRTP(robot, 67, .8, "backward");
        encodersMoveStrafe(robot, 10, .5, "left");
        stoneOnFoundation(robot);
        resetStoneMechanism(robot);
        encodersMoveStrafe(robot, 5, 0.5, "right");
    }

    public void middle2ndSkyStoneRed(Robot robot){
       encodersMoveRTP(robot, 82, .8, "forward");
       encodersMoveStrafe(robot, 17, .5, "right");
       runIntake(robot, -1.0);
       encodersMoveRTP(robot, 10, .2, "forward");
       double startTouchTime = System.currentTimeMillis();
       while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {}
       runIntake(robot, 0.0);
       robot.setCaptureServoDown();
       robotSleep(500);
       encodersMoveStrafe(robot, 15, .8, "left");
       encodersMoveRTP(robot, 60, .8, "backward");
        resetStoneMechanism(robot);
        encodersMoveStrafe(robot, 5, 0.5, "right");
    }

    public void left2ndSkyStoneRed(Robot robot){

    }

    public void right2ndSkyStoneRed (Robot robot){

    }

    public void rightStoneRed(Robot robot){
        encodersMove(robot, 18, .8, "forward");
        encodersMoveStrafe(robot, 23, .5, "right");
        imuTurn(robot, 25, .5, "left");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 20, .2, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 3500){}
        runIntake(robot, 0.0);
        robot.setCaptureServoDown();
        robotSleep(500);
        liftMotorRTPDriveWithStone(robot);
        encodersMoveRTP(robot, 20, .8, "backward");
        imuTurn(robot, 30, .5, "left");
        encodersMoveStrafe(robot, 20, .5, "left");
        encodersMoveRTP(robot, 59, .8, "backward");
        stoneOnFoundation(robot);
        resetStoneMechanism(robot);
        encodersMoveStrafe(robot, 5, 0.5, "right");

    }

    public void stoneOnFoundation(Robot robot){
        int liftCount = (int)robot.liftCalculateCounts(16);
        robot.setLiftMotorTargetPosition(liftCount);
        robot.setLiftMotorPower(0.8);
        while (opModeIsActive() && robot.getCurrentLiftPosition() < liftCount){}
        robot.setLiftMotorPower(0.0);
        robot.setStoneSwingServoOut();
        robotSleep(1000);
        robot.setCaptureServoUp();
    }

    public void resetStoneMechanism(Robot robot){
        robot.setStoneSwingServoIn();
        robotSleep(500);
        int liftPositionDown = (int)robot.getCurrentLiftPosition() - (int)robot.liftCalculateCounts(16);
        robot.setLiftMotorTargetPosition(liftPositionDown);
        robot.setLiftMotorPower(0.8);
        while (opModeIsActive() && robot.getCurrentLiftPosition() > liftPositionDown){}
        robot.setLiftMotorPower(0.0);
        robot.resetLiftEncoder();
        robot.runLiftWithEncoderRTP();

    }

    public void blueFoundation(Robot robot){
        encodersMoveRTP(robot, 10, .8, "backward"); //robot moves backwards 30 inches
        encodersMoveStrafe(robot, 10, .5, "right"); //robot strafes to the middle of foundation
        encodersMoveRTP(robot, 21, .5, "backward");
        robot.setFoundationGrabberGrabbed(); //foundation servos go down
        robotSleep(2000);
        encodersMoveRTP(robot, 25, .8, "forward"); //robot moves forwards 15 inches
        imuTurn(robot, 70, .6, "left"); //robot turns 90 degrees right
        //robotSleep(1000);
        robot.setFoundationGrabberUnGrabbed(); //foundation servos come up
        robotSleep(2000);
        encodersMoveRTP(robot, 30, .8, "backward"); //robot moves to wall
        encodersMove(robot, 5, .8, "forward");
        encodersMoveStrafe(robot, 8, .5, "right");
        encodersMoveRTP(robot, 30, .8, "forward"); //robot parks under SkyBridge
    }

    public void redFoundation(Robot robot) {
        encodersMoveRTP(robot, 10, .8, "backward"); //robot moves backwards 30 inches
        encodersMoveStrafe(robot, 10, .5, "left"); //robot strafes to the middle of foundation
        encodersMoveRTP(robot, 21, .5, "backward");
        robot.setFoundationGrabberGrabbed(); //foundation servos go down
        robotSleep(2000);
        encodersMoveRTP(robot, 25, .8, "forward"); //robot moves forwards 15 inches
        imuTurn(robot, 70, .6, "right"); //robot turns 90 degrees right
        //robotSleep(1000);
        robot.setFoundationGrabberUnGrabbed(); //foundation servos come up
        robotSleep(2000);
        encodersMoveRTP(robot, 30, .8, "backward"); //robot moves to wall
        encodersMove(robot, 5, .8, "forward");
        encodersMoveStrafe(robot, 8, .5, "left");
        encodersMoveRTP(robot, 30, .8, "forward"); //robot parks under SkyBridge
    }

    public void parkSkyBridge(Robot robot){
        encodersMoveRTP(robot, 38, .5, "forward");
    }


    /**
     * Will keep
     * @param robot
     * @param distance
     * @param power
     * @param intendedHeading
     * @param direction
     */
    public void strafingStraight (Robot robot, double distance, double power, double intendedHeading, String direction) {
        double power_adjustment;
        double lPower;
        double rPower;
        robot.resetLiftEncoder();
        robot.runWithEncoders();

        final double v = robot.driveTrainCalculateCounts(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, -power, power); //start strafing right

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                power_adjustment = (robot.getIntegratedZAxis() - intendedHeading) / 100.0;

                if (power_adjustment > 0.05) {
                    power_adjustment = 0.05;
                }

                lPower = power + power_adjustment; //left motor power adjustment
                rPower = power - power_adjustment; // Right motor power adjustment

                robot.setDriveMotorPower(lPower, -rPower, -lPower, rPower);


                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.addData("Left Power", lPower).addData("Right Power", rPower);
                telemetry.update();
            }
        }
    }

}


