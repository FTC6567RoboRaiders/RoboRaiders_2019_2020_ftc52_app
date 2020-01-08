package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.Robot.Robot;

@Autonomous
@Disabled
public class BlueFoundationAuto extends RRAutonomousMethods {

    Robot robot = new Robot();

    @Override
    public void runOpMode(){

        robot.initialize(hardwareMap);
        waitForStart();

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
}
