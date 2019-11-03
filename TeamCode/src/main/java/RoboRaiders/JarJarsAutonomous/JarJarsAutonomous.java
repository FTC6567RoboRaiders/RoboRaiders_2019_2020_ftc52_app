package RoboRaiders.JarJarsAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethods.JarJarAutonomousMethods;
import RoboRaiders.Robot.JarJarBot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous (name= "Movement Test For JarJar")

public class JarJarsAutonomous extends JarJarAutonomousMethods {

    JarJarBot robot = new JarJarBot();

    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"JarJar");

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        rtd.displayRobotTelemetry("Initialized, Waiting For Start");

        waitForStart();

        //encodersMove( robot , 12,0.5, "forward" );

        encodersMoveStrafe(robot,24,.5,"right");

        robotSleep(250);

        encodersMove(robot, 10,.5, "forward");

        robotSleep(250);

        encodersMoveStrafe();
        encodersMoveStrafe(robot,24,.5,"left");
        encodersMove(robot, 12, .5, "forward");
        runIntake(robot);




    }


}

