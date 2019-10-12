package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.AutoOptions.AutoOptions;
import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Logger.Logger;
import RoboRaiders.Robot.NostromoBot;
import RoboRaiders.Robot.NostromoBotMotorDumper;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous
@Disabled
public class StevesStrafing extends NostromoAutonomousMethods{

    public NostromoBotMotorDumper robot = new NostromoBotMotorDumper();




    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);

        // Wait for start to be pushed
        waitForStart();

        while (opModeIsActive()) {
          //  encodersMove(robot,24,1,"right");
        }
    }
}
