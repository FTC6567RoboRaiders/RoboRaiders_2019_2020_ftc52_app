package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID;
import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Robot.NostromoBot;
import RoboRaiders.Robot.NostromoBotMotorDumper;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous(name="Auto: Robot Starts Towards Crater")
@Disabled

public class AutoCrater extends NostromoAutonomousMethods {

    NostromoBotMotorDumper robot = new NostromoBotMotorDumper();
    RoboRaidersPID robotPID = new RoboRaidersPID(0,0,0);
    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"Nostormo");
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        rtd.displayRobotTelemetry("Initialized, Waiting For Start");

        waitForStart();

        rtd.displayRobotTelemetry("Started");
        rtd.displayRobotTelemetry("Deploying from Lander","Calling DeployRobot");

        DeployRobot(robot);

        //closeRedDepot(robotPID, robot, rtd);

        //moveTest(robotPID, robot);

    }

    }

