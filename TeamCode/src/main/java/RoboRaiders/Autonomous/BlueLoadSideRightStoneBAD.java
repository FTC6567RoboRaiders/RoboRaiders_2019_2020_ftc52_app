package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.Robot.Robot;

@Autonomous
@Disabled
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





