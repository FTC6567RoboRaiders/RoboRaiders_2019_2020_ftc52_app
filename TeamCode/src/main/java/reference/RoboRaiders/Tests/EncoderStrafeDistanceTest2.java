//package RoboRaiders.Tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
//import RoboRaiders.Robot.NostromoBotMotorDumper;
//import RoboRaiders.Robot.RobotTelemetryDisplay;
//
//
//@Autonomous (name="Encoder Strafe Distance Test 2", group="Test")
//@Disabled
///**
// * Created by Steve Kocik as a sample for RedStorm to build off of
// */
//
//public class EncoderStrafeDistanceTest2 extends NostromoAutonomousMethods {
//
//
//    NostromoBotMotorDumper robot = new NostromoBotMotorDumper();
//    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"Nostromo");
//    @Override
//    public void runOpMode() {
//
//
//        // Initialize and set up the robot's drive motors
//        robot.initialize(hardwareMap);             // Initialize the robot
//        robot.resetEncoders();                     // Reset the encoder counts
//        robot.runWithEncoders();                   // Tell the motors to run with encoders
//
//        rtd.displayRobotTelemetry("Initialized, press Start to run strafing test");
//
//        // Wait for the start button to be pushed
//        waitForStart();
//
//
//
//        // So somebody did some math some time ago that said:
//        // "With a mecanum drivetrain is driving sideways (along the Y axis below),
//        // there are four force vectors (one from each wheel) directed at 45 degree angles:
//        // The Y components of the force vectors make the robot move in the Y direction, and are
//        // each equal to sin(45)F=0.707F. The X components, which are also equal to .707*F, cancel
//        // each other out.
//        //
//        // Therefore, the robot will have .707 the force driving its sideways movement as it will
//        // driving its forward movement."
//        //
//        // Now .707 is really 1/SQRT(2) - since this is a 45, 45, 90 triangle so one should divide
//        // the distance to travel by 1/SQRT(2) to compensate for the loss of force, thus distance
//        // so if one wants to go 12 inches
//        //
//        // So the formula is:
//        //
//        // D / (1/SQRT(2)) or simplifying: D * SQRT(2)
//
//        double distance         = 12.0;                          // robot is to travel 12 inches
//        double adjustedDistance = Math.sqrt(2.0) * distance;     // adjust distance for loss of force
//
//        rtd.displayRobotTelemetry("Strafing");
//        rtd.displayRobotTelemetry("Distance",String.valueOf(adjustedDistance));
//        rtd.displayRobotTelemetry("Direction","Right");
//        rtd.displayRobotTelemetry("Power","0.5");
//
//        if (opModeIsActive()) {
//
//            encodersMoveStrafe(robot,adjustedDistance,0.5,"right");
//
//        }
//
//        rtd.displayRobotTelemetry("Stop");
//        rtd.displayRobotTelemetry("Strafing complete, measure distance");
//
//    }
//}