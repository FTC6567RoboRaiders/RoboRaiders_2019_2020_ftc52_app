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

        //runIntake(robot,-.5);
        //encodersMove(robot,36,.5,"forward");
        collectStone(robot);
        encodersMove(robot, 70,.5,"forward");
        encodersMoveStrafe(robot,36,.5,"left");




    }


}

