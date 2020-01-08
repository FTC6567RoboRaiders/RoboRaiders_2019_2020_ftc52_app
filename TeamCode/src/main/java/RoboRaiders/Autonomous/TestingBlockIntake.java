package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.Robot.Robot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous (name= "Testing Block Intake")
@Disabled
public class TestingBlockIntake extends RRAutonomousMethods {

    Robot robot = new Robot();

    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"JarJar");

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        rtd.displayRobotTelemetry("Initialized, Wai ting For Start");

        waitForStart();
        collectStone(robot);
        robotSleep(1000);

        runIntake(robot,0);

        encodersMove(robot, 28, .3,"backward");
        robotSleep(1000);

    }

}

