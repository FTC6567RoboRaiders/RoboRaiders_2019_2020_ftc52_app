//package org.firstinspires.ftc.teamcode;
//
//import android.util.Log;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.Range;
//import com.roboraiders.Robot.RoboRaidersAuto;
//import com.roboraiders.Robot.Robot;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//@Autonomous(name = "Drive Straight", group = "Verify")
//@Disabled
//
///**
// * This class will verify that the drive motor encoders are working properly by running with encoders
// * and then outputting the encoder counts to the phone and log.
// *
// * Created by SteveKocik on 11/20/2017.
// */
//
//public class DriveStraight extends RoboRaidersAuto {
//
//    // The following variables are used to control how often telemetry data is written to the log
//    //
//    //  - currentTimeStamp - is the current time stamp, this is updated every time within the
//    //                     while(opModeIsActive()) loop
//    //
//    //  - pastTimeStamp    - is the time stamp that the log was last updated, initially it is set to 0,
//    //                     and is only updated when the log is updated
//    //
//    //  - LOG_INTERVAL     - the amount of time per each log updated, initially set to 1/2 of a
//    //                     second, this value is in milliseconds (1/2 of sec = 500 milliseconds)
//
//    private long currentTimeStamp;
//    private long pastTimeStamp;
//    private static final long LOG_INTERVAL = 500;
//    private boolean itsTimeToLog;
//    private int[] encoderArray = new int[4];
//    private DcMotor.RunMode[] modeArray = new DcMotor.RunMode[4];
//    private float robotCurrentHeading;
//    private double INTENDED_ROBOT_HEADING = 0.0;
//    private double rightPower;
//    private double leftPower;
//
//    public Orientation angles;
//
//
//    public Robot robot = new Robot();
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot.initialize(hardwareMap);
//        pastTimeStamp = 0;
//
//        // Write message to log indicating that teleop program is initialized
//        Log.d("VME", "VerifyMotorEncoders Initialization Complete");
//
//        telemetry.addData("Initialized", true);
//        telemetry.update();
//
//        robot.resetEncoders();                                // resets encoders
//        robot.runWithEncoders();                              // set motors to RUN_WITH_ENCODERS
//
//        waitForStart();
//
//        robot.resetIMU(); //resets IMU angle to zero
//
//
//        while (opModeIsActive()) {
//
//            currentTimeStamp = System.currentTimeMillis();   // get the current time stamp
//            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
//            robotCurrentHeading = angles.firstAngle; //heading is equal to the absolute value of the first angle
//
//
//            leftPower = 0.55 + ((robotCurrentHeading - INTENDED_ROBOT_HEADING) / 100);
//            rightPower = 0.5 - ((robotCurrentHeading - INTENDED_ROBOT_HEADING) / 100);
//
//            leftPower = Range.clip(leftPower, -1, 1);
//            rightPower = Range.clip(rightPower, -1, 1);
//
//
//            robot.setDriveMotorPower(leftPower, rightPower, leftPower, rightPower);    // run the motors at 1/2 speed, don't need fast and furious part 2
//
//            // store the current encoder counts (positions) for the drive motors
//            encoderArray[0] = robot.motorFrontLeft.getCurrentPosition();
//            encoderArray[1] = robot.motorFrontRight.getCurrentPosition();
//            encoderArray[2] = robot.motorBackLeft.getCurrentPosition();
//            encoderArray[3] = robot.motorBackRight.getCurrentPosition();
//
//            modeArray[0] = robot.motorFrontLeft.getMode();
//            modeArray[1] = robot.motorFrontRight.getMode();
//            modeArray[2] = robot.motorBackLeft.getMode();
//            modeArray[3] = robot.motorBackRight.getMode();
//
//
//            // The method timeToLog() will determine if the logging interval has expired from the
//            // last time
//            itsTimeToLog = timeToLog();
//
//            // Log the encoder count for each of the motors
//            if (itsTimeToLog) {
//
//                // Log is an Android class that is used for sending log output to the log file
//                // on the Android phone (in this case the Robot Controller).  There
//                // are several methods that Log supports.  The d method is for logging
//                // debug information.  More information can be found at:
//                // https://developer.android.com/reference/android/util/Log.html
//                Log.d("VME", "********************************************************");
//                Log.d("VME", "Start of Encoder Counts for Drive Motors");
//                Log.d("VME", String.format("motorFrontLeft:  %s", encoderArray[0]));
//                Log.d("VME", String.format("motorFrontRight: %s", encoderArray[1]));
//                Log.d("VME", String.format("motorBackLeft:   %s", encoderArray[2]));
//                Log.d("VME", String.format("motorBackRight:  %s", encoderArray[3]));
//                Log.d("VME", "End of Encoder Counts for Drive Motors");
//                Log.d("VME", "********************************************************");
//
//                // Update the driver station display with the same information as has been
//                // captured to the log file.
//                telemetry.addLine().addData("motorFrontLeft:  ", encoderArray[0]).addData("EncMode: ", modeArray[0]);
//                telemetry.addLine().addData("motorFrontRight: ", encoderArray[1]).addData("EncMode: ", modeArray[1]);
//                telemetry.addLine().addData("motorBackLeft:   ", encoderArray[2]).addData("EncMode: ", modeArray[2]);
//                telemetry.addLine().addData("motorBackRight:  ", encoderArray[3]).addData("EncMode: ", modeArray[3]);
//                telemetry.addLine().addData("heading:         ", robotCurrentHeading);
//
//
//                telemetry.update();
//
//            } // if( itsTimeToLog )
//
//        } // while(opModeIsActive())
//
//        robot.setDriveMotorPower(0, 0, 0, 0);                 // stop the robot
//        robot.resetEncoders();                                // stop and reset the encoders
//
//    } // public void runOpMode() throws InterruptedException
//
//    /**
//     * Will determine when the log should be updated with new data.  The previous time is subtracted
//     * from the current time with a result of a time change or delta.  The time delta is then compared
//     * to the log interval (LOG_INTERVAL) which represents the number of seconds (or fractions of a
//     * second) that should expire before updating the log.  If the delta time is greater than the
//     * log interval, this method will return a true.  If the delta time is less than the log interval,
//     * this method will return a false.
//     * <p>
//     * Under the covers, this method will set the variable pastTimeStamp, when the log interval time
//     * has expired.
//     *
//     * @return boolean - TRUE, interval has expired and caller should write to log
//     * - FALSE, interval has not expired and caller should not write to log
//     */
//
//    private boolean timeToLog() {
//
//        if ((currentTimeStamp - pastTimeStamp) > LOG_INTERVAL) {
//            pastTimeStamp = currentTimeStamp;
//            return true;
//        } else {
//            return false;
//        }
//
//    } // private boolean timeToLog()
//
//
//}
//
