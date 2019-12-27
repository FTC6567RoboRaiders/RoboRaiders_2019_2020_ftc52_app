package RoboRaiders.JarJarsAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.Robot.Robot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous (name= "Movement Test For BotLord")

public class JarJarsAutonomous extends RRAutonomousMethods {

    Robot robot = new Robot();

    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"JarJar");

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        rtd.displayRobotTelemetry("Initialized, Waiting For Start");

        waitForStart();
        encodersMoveStrafe(robot,20,0.4, "left");
        robotSleep(500);
        encodersMoveStrafe(robot,15,0.4,"right");
        robotSleep(500);
        encodersMoveStrafe(robot,10,0.4,"left");
        robotSleep(500);
        encodersMoveStrafe(robot,5,0.4,"right");




    }


}

