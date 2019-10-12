package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID;
import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Robot.NostromoBotMotorDumper;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous
@Disabled
public class NewPIDTurnTest extends NostromoAutonomousMethods {
    public NostromoBotMotorDumper robot = new NostromoBotMotorDumper();



    private RoboRaidersPID rrPID;
    private RobotTelemetryDisplay rtd;
    private double kP, kI, kD, direction, degrees = 45.0;



    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        // Create new instance of robot telemetry display
        //rtd = new RobotTelemetryDisplay(this,"Nostromo");


        //Create new rrPID
        rrPID = new RoboRaidersPID(0.02,0,0.1);




        // initialize the robot
        robot.initialize(hardwareMap);



        // Wait for start to be pushed
        waitForStart();

        if (opModeIsActive()) {



                imuTurnPID(rrPID, robot, (float)degrees, String.valueOf("right"));

        }

    }

}


