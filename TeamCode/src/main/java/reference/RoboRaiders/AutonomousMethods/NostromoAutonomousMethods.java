package RoboRaiders.AutonomousMethods;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID;
import RoboRaiders.Logger.Logger;
import RoboRaiders.Robot.NostromoBot;
import RoboRaiders.Robot.NostromoBotMotorDumper;
import RoboRaiders.Robot.RobotTelemetryDisplay;


public abstract class NostromoAutonomousMethods extends LinearOpMode {

    public double motor_power;
    public double degreesToTurn;
    public double currentHeading;
    public double finalHeading;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;


    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this, "Nostormo");

    /*public void farRedDepot(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        EncoderDrivePID( robot, 50, "forward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(50));
        rtd.displayRobotTelemetry("Encoder Count", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

        imuTurn(robot, 90, .25, "right");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(90));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(500);

        //DeployTeamMarker(robot);
        rtd.displayRobotTelemetry("Deploying Team Marker");

        encodersMove(robot, 6, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(6));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);


        imuTurn(robot, 60, .25, "left");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(500);

        encodersMove(robot, 30, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(30));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

        imuTurn(robot, 105, .35, "right");  // was 100

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(105));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        encodersMove(robot, 15, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(15));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(15));

    }

    public void closeRedDepot(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        //DeployRobot(robot);

        EncoderDrivePID(robot, 28, "forward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(28));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(200);

        encodersMove(robot, 3, 1, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(200);

        imuTurn(robot, 100, .35, "left");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To  Be Turned", String.valueOf(100));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(100);

        EncoderDrivePID(robot, 41, "forward");   // was 40 now 41

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(41));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 3.0, 0.25, "forward"); // was 1

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        imuTurn(robot, 60, .35, "left");  // was 55

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        EncoderDrivePID(robot, 36, "forward"); // was 39

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(36));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 1, 0.25, "forward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(1));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        robot.collectionOff();
        rtd.displayRobotTelemetry("Turning Off Collection");

        imuTurn(robot, 105, .35, "right");  // was 100

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(105));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        // DeployTeamMarker(robot);
        rtd.displayRobotTelemetry("Deploying Team Marker");

        imuTurn(robot, 90, .35, "right"); // was 85, now at 90

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(90));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(200);

        EncoderDrivePID(robot, 68, "forward");  // was 78 inches

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(68));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(250);
    }

    public void farBlueDepot(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        EncoderDrivePID( robot, 28, "forward");
        robotSleep(500);

        encodersMove(robot, 6, .80, "backward");
        robotSleep(500);

        imuTurn(robot, 90, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robot, 35, "forward");
        robotSleep(500);

        imuTurn(robot, 40, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robot, 34, "forward");
        robotSleep(500);

        imuTurn(robot, 90, .25, "right");
        robotSleep(500);

        imuTurn(robot, 90, .30, "right");
        robotSleep(500);

        EncoderDrivePID(robot, 78, "forward");
        robotSleep(500);
    }

    public void closeBlueDepot(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        EncoderDrivePID (robot, 48, "forward");
        robotSleep(500);

        imuTurn(robot, 90, .25, "left");
        robotSleep(500);

        EncoderDrivePID( robot, 1, "forward");
        robotSleep(500);

        imuTurn(robot, 45, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robot, 60, "forward");
        robotSleep(500);
    }

    public void moveTest(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        EncoderDrivePID(robot, 48, "forward");
    }

    public void moveDepotFromCraterStart(NostromoBotMotorDumper robot) {


        EncoderDrivePID(robot, 35, "forward");   // was 40 now 41

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Moving Forward", String.valueOf(41));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 3.0, 0.25, "forward"); // was 1

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Count", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        imuTurn(robot, 65, .35, "left");  // was 55

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        EncoderDrivePID(robot, 36, "forward"); // was 39

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward");
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 1, 0.25, "forward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(1));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        imuTurn(robot, 30, .35, "right");


    }

    public void moveDepotFromDepotStart(NostromoBotMotorDumper robot) {
        encodersMove(robot, 6,0.8,"Forward");//also mineral knocking was 50

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(6));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

    }
*/
    public void parkFromCraterStart(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot) {

        encodersMovePID(drivePID, robot, 43,  "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(68));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));

        imuTurn(robot, 4, .45, "left");
        robotSleep(250);

        encodersMovePID(drivePID, robot, 15,  "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(68));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        //robotSleep(250);

        //encodersMove(robot, 10, 0.5, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(68));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        //robotSleep(250);
    }

    public void parkFromDepotStart(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot) {

       //imuTurn(robot, 5, .5, "right");
       //encodersMovePID(drivePID, robot, 48.0,"backward");
       encodersMovePID(drivePID, robot, 43.0,"backward");
       //robotSleep(500);

       imuTurn(robot, 20, .45, "right");
       robotSleep(500);
       encodersMove(robot, 5, .45,"backward");
        /*
       imuTurnPID(turnPID, robot, 170,  "right");
       encodersMovePID(drivePID, robot, 10.0, "forward");
       double intakeCOUNTS = Math.abs(robot.calculateCOUNTSREVMotor(3.5));
       robot.resetIntakeEncoders();
       robot.runIntakeWithEncoders();

       robot.setLiftIntakePower(.5);

       while(opModeIsActive() && robot.getIntakeEncoderCounts() < intakeCOUNTS){}

       robot.setLiftIntakePower(0.0);

       robot.runIntakeWithoutEncoders();

       //robot.setMotorDrawerSlide(-0.75);
       //robotSleep(2000);
       //robot.setMotorDrawerSlide(0);
       */


       //encodersMovePID(drivePID, robot, 6,  "backward");
    }



    /**
     * Tuner method to tune the PID variables for driving straight
     * @param robotPID
     * @param robot
     * @param wantedDistance
     * @param direction
     */
    public void encoderDrivePIDTuner(RoboRaidersPID robotPID, NostromoBotMotorDumper robot, double wantedDistance, double direction) {


        robot.resetEncoders();
        robot.runWithEncoders();
        robotPID.initialize();   // re-initialized the pid variables that we care about

        double EncoderCount = Math.abs(robot.calculateCOUNTS(wantedDistance));
        double currentEncoderCount = robot.getSortedEncoderCount();
        if (direction == 0.0) {
        while (opModeIsActive() && (currentEncoderCount <= EncoderCount || currentEncoderCount >= EncoderCount ))
                {
                    motor_power = robotPID.CalculatePIDPowers(EncoderCount, robot.getSortedEncoderCount());
                        robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);
                        currentEncoderCount = robot.getSortedEncoderCount();
                    }

        }
        else if (direction == 1.0) {
            while (opModeIsActive() && (currentEncoderCount <= EncoderCount || currentEncoderCount >= EncoderCount ))
            {
                motor_power = robotPID.CalculatePIDPowers(EncoderCount, robot.getSortedEncoderCount());
                robot.setDriveMotorPower(-motor_power, -motor_power, -motor_power, -motor_power);
                currentEncoderCount = robot.getSortedEncoderCount();
            }

        }
    }

    /* public void imuTurnWithPID (NostromoBot robot, float degrees, String direction ) {
         robot.resetIMU();
         float finalHeading = robot.getHeading() + degrees;

         // robot.getHeading(); returns the current heading of the IMU

         if (direction.equals("right")) { //if the desired direction is right

             robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
         }
         else if (direction.equals("left")) { //if the desired direction is left

             robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn left
         }

         while (robot.getHeading() < (finalHeading - 20) && opModeIsActive()) { //while the value of getHeading is
             //less then the degree value
             //and while opMode is active continue the while loop

             telemetry.addData("Heading", robot.getHeading()); //feedback of getHeading value
             telemetry.update(); //continuous update
         }

         robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
     }
     }*/
   /* public void EncoderDrivePID(NostromoBotMotorDumper robot, double wantedDistance, double direction) {
        //    robot.resetEncoders();
        //    robot.runWithEncoders();
        RoboRaidersPID pidClass = new RoboRaidersPID();   // create new pidClass

        EncoderDrivePID(pidClass, robot, wantedDistance, direction);*/



    public void imuTurn(NostromoBotMotorDumper robot, float degreesToTurn, double power, String direction) { //gets hardware from
        //Robot and defines degrees as a
        //float, power as a double, and direction as a string
        //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addLine().addData("degreesToTurn",String.valueOf(degreesToTurn));
        currentHeading = robot.getIntegratedZAxis();
        //finalHeading = currentHeading + degreesToTurn;
        //telemetry.update();

        // robot.getHeading(); returns the current heading of the IMU

        if (direction.equals("right")) { //if the desired direction is right
            finalHeading = currentHeading - degreesToTurn;
            robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
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
            robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn left
            while(opModeIsActive() && robot.getIntegratedZAxis() < finalHeading) {
                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                //telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                //telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                //telemetry.update();
            }
        }


        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
    }

    /**
     * turning with PID
     * @param robot
     * @param rrPID
     * @param degreesToTurn
     * @param direction
     */
    public void imuTurnPID(RoboRaidersPID rrPID, NostromoBotMotorDumper robot, float degreesToTurn, String direction) { //gets hardware from
        double power = 0.0;
        int loopcount = 0;


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

        robot.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);


        L.Debug("End");
    }


    /**
     * This will lower the robot from the lander and opens the claw
     * @param robot
     */
    public void DeployRobot(NostromoBotMotorDumper robot) {

        double startDeployTime = System.currentTimeMillis();
        //lower at full power until either the touch sensor is touched, or 1.5 seconds is elapsed

        robot.setLiftMotorPower(-0.95);
        while (opModeIsActive() && System.currentTimeMillis() - startDeployTime < 1500 && !robot.sensorTouch.isPressed()) {
        }

        //as long as its after 1.5 seconds, less than 5 seconds, and the touch sensor isn't pressed, go at half speed down
        while (opModeIsActive() && System.currentTimeMillis() - startDeployTime >= 1500 &&
                System.currentTimeMillis() - startDeployTime < 5000 && !robot.sensorTouch.isPressed()) {
            robot.setLiftMotorPower(-.45);
        }

        robot.setLiftMotorPower(0);

        robot.liftClaw.setPosition(robot.liftClawOpen);
        //open that claw
        robotSleep(1200);
        //Maybe we can eliminate this sleep

    }

    public void DeployTeamMarker(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot, boolean startLocation) {
         /*long t = System.currentTimeMillis();
        long end = t + 500;
        while (System.currentTimeMillis() < end) {//up
            robot.motorDumpp.setPower(-0.7);
        }



        robot.motorDumpp.setPower(0); */

        if (startLocation) {//are we starting from the crater?)
            imuTurnPID(turnPID, robot, 20,  "right");
        }

        robot.dumpWrist.setPosition(robot.dropTeamMarker);//put elbow down
        robotSleep(1000);

         /*while (System.currentTimeMillis() < end) {//down
            robot.motorDumpp.setPower(0.7);
        }
        robot.motorDumpp.setPower(0);

        while (System.currentTimeMillis() < end) { //up
            robot.motorDumpp.setPower(-0.7);
        }
        robot.motorDumpp.setPower(0);
        */
        robot.dumpWrist.setPosition(robot.bringMarkerBack);
        robotSleep(500);

        if (startLocation) {//are we starting from the crater?)
            imuTurnPID(turnPID, robot, 20, "left");
        }


    }

    /**
     * * Will detect the location of the gold mineral
     * @param drivePID
     * @param turnPID
     * @param robot
     */
    public void samplingMineralsDepot(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot, boolean sampleWait) {


        encodersMove(robot, 2, .45, "forward");
        //robotSleep(200);

        imuTurnPID(turnPID, robot, 90,"left");

        encodersMove(robot, 2.5, .4, "forward");


        int goldLocation = detectGoldMineral(robot);

        telemetry.addLine().addData("Mineral Seen", String.valueOf(goldLocation));
        telemetry.update();


        if (sampleWait){
            RRsleep(5000);
        }


        switch (goldLocation) {

            case 1:
                mineralLeftDepot(drivePID, turnPID, robot);
                break;
            case 2:
            case -1:
                mineralCenterDepot(drivePID, turnPID, robot);
                break;
            case 3:
                mineralRightDepot(drivePID, turnPID, robot);
                break;

        }
    }

    /**
     * * Will detect the location of the gold mineral
     * @param drivePID
     * @param turnPID
     * @param robot
     */
    public void samplingMineralsCrater(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot, boolean sampleWait) {

        encodersMove(robot, 2, .45,"forward");
        robotSleep(200);

        imuTurnPID(turnPID, robot,90, "left");

        encodersMove(robot, 2.5, .45,"forward");

        int goldLocation = detectGoldMineral(robot);
        telemetry.addLine().addData("GoldLocation", goldLocation);


        {
            if (sampleWait) {
                RRsleep(5000);
            }
        }

        switch (goldLocation) {
            case 1:
                mineralLeftCrater(drivePID, turnPID, robot);
                break;
            case 2:
                mineralCenterCrater(drivePID, turnPID, robot);
                break;
            case 3:
            case -1:
                mineralRightCrater(drivePID, turnPID, robot);
                break;
        }
    }

    /**
     * will detect the gold mineral location
     *
     * @return the location of the gold mineral
     */
    public int detectGoldMineral(NostromoBotMotorDumper robot) {
        int goldPostion = -1;
        int numberofrecognized = 0;
        List<Recognition> updatedRecognitions = null;
        double prevGoldConfidence = 0.0;
        Logger  L = new Logger("detectGoldMineral");
        int numberOfGoldDetected = 0;


        robotSleep(500);

        CameraDevice.getInstance().setFlashTorchMode(true);

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */

            if (robot.tfod != null) {
                robot.tfod.activate();
            }

            //took out turning on the flash for the second time
            //took out a 1.5 second wait
            double startSamplingTime = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - startSamplingTime <= 2500 && numberofrecognized < 2) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions == null) {
                    numberofrecognized = 0;
                } else {
                    numberofrecognized = updatedRecognitions.size();
                }

            }//while?
            L.Debug("numberRecognized", numberofrecognized);

            if (updatedRecognitions == null) {
                telemetry.addLine("updatedRecognitionsIsNull");
                L.Debug("No recognitions found");
            } else {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // Is the robot seeing at least one mineral  THIS IS CHANGED, WE NOW JUST CARE IF WE SEE AT LEAST ONE THING
                if (updatedRecognitions.size() >= 1) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
                            numberOfGoldDetected++;
                            L.Debug("numberOfGoldDetected", numberOfGoldDetected);
                            L.Debug("goldConfidence", recognition.getConfidence());
                            L.Debug("gold position ",recognition.getLeft());
                            if (prevGoldConfidence < recognition.getConfidence()) {
                                goldMineralX = (int) recognition.getLeft();
                                prevGoldConfidence = recognition.getConfidence();

                            }
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                            L.Debug("silverConfidence", recognition.getConfidence());
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                            L.Debug("silverConfidence", recognition.getConfidence());
                        }
                    }
                    telemetry.addData("goldMineralX", String.valueOf(goldMineralX));
                    telemetry.addData("silverMineral1X", String.valueOf(silverMineral1X));
                    telemetry.addData("silverMineral2X", String.valueOf(silverMineral2X));

                    L.Debug("goldMineralX", goldMineralX);
                    L.Debug("silverMineral1X", silverMineral1X);
                    L.Debug("silverMineral2X", silverMineral2X);
                    L.Debug("numberofGoldDetected",numberOfGoldDetected);


                    //subtract one gold detected to not fall into mineralRight
                    //when number of gold detected > 1 we have found more than one gold mineral one is false
                    if (numberOfGoldDetected > 1){
                        numberofrecognized--;
                    }
                    L.Debug("numberofGoldDetected",numberOfGoldDetected);


                    // Did the robot not see the gold mineral    THIS IS CHANGED
                    if (goldMineralX == -1 || numberofrecognized == 3) {

                        // Yes, indicate the gold mineral is on the right
                        goldPostion = 3;
                    }
                    // The robot saw a gold mineral find where it is
                    else {

                        // Is the gold mineral to the left of the silver mineral?  THIS IS CHANGED
                        if (goldMineralX > -1 && goldMineralX < silverMineral1X) {

                            // Yes, indicate the gold mineral is in the left
                            goldPostion = 1;
                        } else {

                            // No, the gold mineral is on the center
                            goldPostion = 2;
                        }
                    }
                }
                telemetry.update();
                L.Debug("goldPosition", goldPostion);
            }

        }

        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }
        return goldPostion;

    }



    public void mineralLeftDepot(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot) {

        imuTurnPID(turnPID, robot, 70, "right"); //turn towards mineral
        //robotSleep(500);

        encodersMovePID(drivePID, robot, 43, "forward"); //push the mineral
        //robotSleep(500);

        encodersMove(robot, 3, .5, "backward"); //pull back
        //robotSleep(500);

        imuTurnPID(turnPID, robot, 73,  "right"); //turn towards depot was 60
        //robotSleep(500);

        encodersMove(robot, 5, .5,"forward"); //ready to deploy team marker
        //robotSleep(500);

    }



    public void mineralRightDepot (RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot) {

        encodersMove(robot, 2, .45, "backward");

        imuTurnPID(turnPID, robot, 125, "right"); //turn towards mineral
        //robotSleep(250);

        encodersMovePID(drivePID, robot, 39,  "forward"); //push the mineral
        //robotSleep(250);

        encodersMovePID(drivePID, robot, 3,  "backward"); //push the mineral
        //robotSleep(250);

        imuTurnPID(turnPID, robot, 80,  "left"); //turn towards depot
        //robotSleep(250);

        encodersMovePID(drivePID, robot, 34,  "forward"); //ready to deploy team marker
       //robotSleep(250);

        imuTurnPID(turnPID, robot, 84, "right"); //turn towards depot
        //robotSleep(250);

        encodersMove(robot, 9, 0.5,"backward"); //ready to deploy team marker
       // robotSleep(250);

    }

    public void mineralCenterDepot(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot)  {

        encodersMove(robot, 2, .45, "backward");
        robotSleep(200);

        imuTurnPID(turnPID, robot,87,"right");

        encodersMovePID(drivePID, robot, 52, "forward");
       // robotSleep(200);

        imuTurnPID(turnPID, robot,45,"right");
        //robotSleep(200);

        encodersMovePID(drivePID, robot,15,"backward");
        //robotSleep(200);


    }


    /////crater thing now

    public void  mineralLeftCrater(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot) {

        //encodersMove(robot, 8, .5, "forward");
        //robotSleep(500);

        imuTurnPID(turnPID, robot,65, "right");
        //robotSleep(250);

        encodersMovePID(drivePID, robot,22, "forward");
        //robotSleep(250);

        encodersMovePID(drivePID, robot,5,"backward");
        //robotSleep(250);

        imuTurnPID(turnPID, robot, 48,"left");
        //robotSleep(250);

        encodersMovePID(drivePID, robot, 26,"forward");
        //robotSleep(250);

        imuTurnPID(turnPID, robot, 60,  "left");
        //robotSleep(250);

        encodersMovePID(drivePID, robot,37,"forward");
        //robotSleep(250);

    }

    public void mineralRightCrater(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot) {

        encodersMove(robot,3,.45,"backward");

        imuTurnPID(turnPID, robot, 115, "right");
        //robotSleep(250);

        encodersMovePID(drivePID, robot, 25, "forward");
        //robotSleep(250);

        encodersMovePID(drivePID, robot, 15, "backward");
        //robotSleep(250);

        imuTurnPID(turnPID, robot, 100, "left");
        //robotSleep(250);

        encodersMovePID(drivePID, robot, 45, "forward");
        //robotSleep(250);

        imuTurnPID(turnPID, robot, 60,"left");
        //robotSleep(250);

        encodersMovePID(drivePID, robot,38,"forward");
        //robotSleep(250);

    }

    public void mineralCenterCrater(RoboRaidersPID drivePID, RoboRaidersPID turnPID, NostromoBotMotorDumper robot)  {

        encodersMove(robot, 3, .45, "backward");

        imuTurnPID(turnPID, robot, 85,"right");
        //robotSleep(200);

        encodersMovePID(drivePID, robot, 20, "forward");
        //robotSleep(250);

        encodersMove(robot, 9, .45, "backward");
        //robotSleep(250);

        imuTurnPID(turnPID, robot, 82, "left");
        //robotSleep(250);

        encodersMovePID(drivePID, robot,43, "forward");
        //robotSleep(250);

        imuTurnPID(turnPID, robot, 55, "left");
        //robotSleep(250);

        encodersMovePID(drivePID, robot,35,"forward");
        //robotSleep(250);
    }







    /*  public void DistanceDrivePID() {
        while (opModeIsActive() && robot.getSensorDistance() < Target) {
            motor_power = drivePID.pidWithDistance(robot.getSensorDistance(), Target);
            robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);

            telemetry.addData("Distance Sensor", robot.getSortedEncoderCount());
            telemetry.addData("Target Distance", Target);
            telemetry.update();

        }*/
    public void encodersMove(NostromoBotMotorDumper robot, double distance, double power, String direction) { //sets the parameters

        robot.resetEncoders(); //resets encoders
        robot.runWithEncoders(); //sets the mode back to run with encoder

        final double v = robot.calculateCOUNTS(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("forward")) { //if the desired direction is forward

            robot.setDriveMotorPower(power, power, power, power); //start driving forward

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                //telemetry.addData("COUNTS", COUNTS);
                //telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                //telemetry.update();
            }

            robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        } else if (direction.equals("backward")) { //if the desired direction is backward

            robot.setDriveMotorPower(-power, -power, -power, -power); //start driving backward

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        } else if (direction.equals("right")) { //if the desired direction is right

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

    /**
     * Tuner method to tune the PID variables for driving straight
     * @param robotPID
     * @param robot
     * @param distance
     * @param direction- "forward" drive forward, "backward" drive backwards
     */
    public void encodersMovePID(RoboRaidersPID robotPID, NostromoBotMotorDumper robot, double distance, String direction) {


        double power = 0.5;

        robot.resetEncoders();
        robot.runWithEncoders();
        robotPID.initialize();   // re-initialized the pid variables that we care about
        //motor_power = 0.5;

        double EncoderCount = Math.abs(robot.calculateCOUNTS(distance));
        double error = Math.abs(robot.calculateCOUNTS(1.0));
        double currentEncoderCount = robot.getSortedEncoderCount(); //we need to play with a range
        if (direction.equals("forward")) {
            while (opModeIsActive() &&
                    !(currentEncoderCount > EncoderCount - error && currentEncoderCount < EncoderCount - error ) &&
                    power > 0.1)
            {
                power = robotPID.CalculatePIDPowers(EncoderCount, robot.getSortedEncoderCount());
                robot.setDriveMotorPower(power, power, power, power);
                currentEncoderCount = robot.getSortedEncoderCount();
            }

        }
        else {
            while (opModeIsActive() &&
                    !(currentEncoderCount > EncoderCount - error && currentEncoderCount < EncoderCount - error ) &&
                    power > 0.1)
            {
                power = robotPID.CalculatePIDPowers(EncoderCount, robot.getSortedEncoderCount());
                robot.setDriveMotorPower(-power, -power, -power, -power);
                currentEncoderCount = robot.getSortedEncoderCount();
            }
        }
    }



    public void encodersMoveStrafe(NostromoBotMotorDumper robot, double distance, double power, String direction) { //sets the parameters

        robot.resetEncoders(); //resets encoders
        robot.runWithEncoders(); //sets the mode back to run with encoder

        final double v = robot.calculateCOUNTS(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power-0.3, -power-0.3, power); //start strafing right

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

    /**
     * make the robot sleep (wait)
     *
     * @param timeToSleep time in milliseconds
     */
    public void robotSleep(int timeToSleep) {
        try {
            Thread.sleep(timeToSleep);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * make the robot sleep (wait)
     *
     * @param timeToSleep time in milliseconds
     */
    public void RRsleep(long timeToSleep) {

        long startTime = System.currentTimeMillis();
        long displayTime = startTime;
        long updatingTime = 1;

        while(System.currentTimeMillis() - startTime < timeToSleep) {
            if(System.currentTimeMillis() - displayTime > (updatingTime * 250)) {
                telemetry.addLine("Patience Kevin!");
                telemetry.update();
                updatingTime++;
            }
        }

    }
}
