package RoboRaiders.JarJarsAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.Robot.Robot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous

public class RedFoundationAuto extends RRAutonomousMethods {

    Robot robot = new Robot();

    @Override
    public void runOpMode(){

        robot.initialize(hardwareMap);
        waitForStart();

        encodersMoveRTP(robot, 10, .8, "backward"); //robot moves backwards 30 inches
        encodersMoveStrafe(robot, 12, .5, "left"); //robot strafes to the middle of foundation
        encodersMoveRTP(robot, 18, .8, "backward");
        robot.setFoundationGrabberGrabbed(); //foundation servos go down
        robotSleep(2000);
        encodersMoveRTP(robot, 20, .8, "forward"); //robot moves forwards 15 inches
        imuTurn(robot, 90, .6, "right"); //robot turns 90 degrees right
        encodersMoveRTP(robot, 20, .8, "backward"); //robot moves to wall
        robot.setFoundationGrabberUnGrabbed(); //foundation servos come up
        robotSleep(2000);
        encodersMoveRTP(robot, 50, .8, "forward"); //robot parks under SkyBridge
    }
}
