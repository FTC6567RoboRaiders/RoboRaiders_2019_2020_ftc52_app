package RoboRaiders.JarJarsAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethods.JarJarAutonomousMethods;
import RoboRaiders.Robot.JarJarBot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous (name= "Testing Block Intake")

public class TestingBlockIntake extends JarJarAutonomousMethods {

    JarJarBot robot = new JarJarBot();

    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"JarJar");

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        rtd.displayRobotTelemetry("Initialized, Waiting For Start");

        waitForStart();

        encodersMove(robot, 10, .3,"forward");
        wait(1000);
        intakeArmDown(robot);
        wait(1000);
        encodersMove(robot, 10, .3, "backward");
        wait(2000);
        runIntake(robot, .6);
        encodersMove(robot, 3, .1, "forward");
        wait(500);
        intakeArmUp(robot);





    }


}

