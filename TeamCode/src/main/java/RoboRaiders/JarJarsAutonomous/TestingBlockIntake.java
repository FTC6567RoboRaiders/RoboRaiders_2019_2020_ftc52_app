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
        robotSleep(1000);

        intakeArmAuto(robot, 0.0);
        robotSleep(1000);

        encodersMove(robot, 2.0,.5,"backward");
        robotSleep(3000);

        encodersMove(robot, 7.0, .4, "forward");


        runIntake(robot, .4);
        robotSleep(3000);


        robotSleep(1000);

        intakeArmAuto(robot, 1.0);





    }


}

