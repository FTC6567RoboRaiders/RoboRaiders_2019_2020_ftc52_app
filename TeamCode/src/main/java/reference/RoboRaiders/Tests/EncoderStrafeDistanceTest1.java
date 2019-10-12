package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Robot.NostromoBotMotorDumper;
import RoboRaiders.Robot.RobotTelemetryDisplay;


@Autonomous (name="Encoder Strafe Distance Test 1", group="Test")
@Disabled

/**
 * Created by Steve Kocik as a sample for RedStorm to build off of...
 */

public class EncoderStrafeDistanceTest1 extends NostromoAutonomousMethods {

    NostromoBotMotorDumper robot = new NostromoBotMotorDumper();
    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"Nostromo");

    @Override
    public void runOpMode() {


        // Initialize and set up the robot's drive motors
        robot.initialize(hardwareMap);             // Initialize the robot
        robot.resetEncoders();                     // Reset the encoder counts
        robot.runWithEncoders();                   // Tell the motors to run with encoders

        rtd.displayRobotTelemetry("Initialized, press Start to run strafing test");

        // Wait for the start button to be pushed
        waitForStart();



        // This is a measurement test, that is, for a given number of rotations of the driver motor,
        // how far does the robot travel.
        //
        // Formula is as follows:
        //
        // Distance = # of Rotations * Wheel Circumference * 1/(Gear Ratio)
        // where:
        // - # of rotations is how many rotations the wheels should turn
        // - Wheel Circumference is PI*Diameter of wheel (PI*4 = 12.566)
        // - 1/(Gear Ratio) the inverse of the gear ratio, so 3/2
        // Number of                             Result
        // Rotations           Calculation       (inches)
        //     1             1 * 12.566 * 3/2     18.849
        //     5             5 * 12.566 * 3/2     94.245
        //    10            10 * 12.566 * 3/2    188.496
        //    20            20 * 12.566 * 3/2    282.735
        //

        rtd.displayRobotTelemetry("Strafing");
        rtd.displayRobotTelemetry("Distance","94.245 in");
        rtd.displayRobotTelemetry("Direction","Right");
        rtd.displayRobotTelemetry("Power","0.5");

        if (opModeIsActive()) {

            encodersMove(robot,94.245,0.5,"right");

        }

        rtd.displayRobotTelemetry("Stop");
        rtd.displayRobotTelemetry("Strafing complete, measure distance");

    }
}