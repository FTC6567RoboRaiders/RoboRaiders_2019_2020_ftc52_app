package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.Robot.Robot;

@Autonomous

public class BlueLoadSideRightStoneBAD extends RRAutonomousMethods {

    Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.initialize(hardwareMap);
        telemetry.addLine("initialized");
        telemetry.update();
        waitForStart();
        rightStoneRed(robot);

    }
}





