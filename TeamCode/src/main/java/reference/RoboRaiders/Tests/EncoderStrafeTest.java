package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoboRaiders.Robot.NostromoBot;


@Autonomous (name="Encoder Strafe Test", group="Test")
@Disabled

/**
 * Created by Steve Kocik as a sample for RedStorm to build off of...
 */

public class EncoderStrafeTest extends LinearOpMode {

    NostromoBot robot = new NostromoBot();

    @Override
    public void runOpMode() {


        // Initialize and set up the robot's drive motors
        robot.initialize(hardwareMap);             // Initialize the robot
        robot.resetEncoders();                     // Reset the encoder counts
        robot.runWithEncoders();                   // Tell the motors to run with encoders

        telemetry.addData("Status ", "Initialized");
        telemetry.update();

        // Wait for the start button to be pushed
        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Status ", "opModeIsActive");


            robot.setDriveMotorPower(0.5, -0.5, -0.5, 0.5);   // Set power to 50%
            telemetry.addData("Left Rear Drive Encoder Counts", "(%.0f)",robot.getBackLeftDriveEncoderCounts());
            telemetry.addData("Left Front Drive Encoder Counts", "(%.0f)",robot.getFrontLeftDriveEncoderCounts());
            telemetry.addData("Right Rear Drive Encoder Counts", "(%.0f)", robot.getBackRightDriveEncoderCounts());
            telemetry.addData("Right Front Drive Encoder Counts", "(%.0f)", robot.getFrontRightDriveEncoderCounts());


            telemetry.update();
        }


        robot.setDriveMotorPower(0.0,0.0, 0.0, 0.0);         // Motors stop
    }
}