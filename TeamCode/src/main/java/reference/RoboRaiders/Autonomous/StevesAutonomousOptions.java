package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.AutoOptions.AutoOptions;
import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Logger.Logger;
import RoboRaiders.Robot.NostromoBot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous(name="Test: Steeeves Auto",group = "Hubbot")
@Disabled

public class StevesAutonomousOptions extends NostromoAutonomousMethods {

    private boolean isRed             = false;
    private boolean startLocation     = false;
    private boolean deployFromLander  = false;
    private boolean claimDepot        = false;
    private boolean parkInCrater      = false;
    private boolean selectionsAreGood = false;
    public NostromoBot robot = new NostromoBot();
    StringBuilder[][] robotVarInfo = new StringBuilder[6][2];


    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {


        // Ask drivers how they want autonomous to work
        AutoOptions myAO = new AutoOptions(this);

        // Set up for logging messages to the log
        Logger L = new Logger(String.valueOf("FTC6567"));

        // Set up robot telemetry
        RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"Nostromo");


        // While the drivers haven't made up their mind, keep asking what they want to do
        while (!selectionsAreGood) {

            isRed            = myAO.selectAlliance();              // Get the alliance (Red or Blue)
            startLocation    = myAO.selectStartLocation();         // Get where the robot is starting from (Depot or Crater)
            deployFromLander = myAO.selectDeployFromLander();      // Should the robot deploy from the lander (Yes or No)
            claimDepot       = myAO.selectClaimDepot();            // Should the robot claim the depot
            parkInCrater     = myAO.selectParkInCrater();          // Should the robot park in crater

            // Add new/additional auto options, so things like drive to depot, drop team marker, etc..


            // Display the options selected
            // We show two options per line, to save space and lines.  The maximum number of characters
            // per line is roughly 45.  Maximum number of lines to be displayed is 9.
            // Note: To keep the autonomous options displayed, the automagical clearing of the telemetry data will be
            //       turned off with the setAutoClear(false) prior to calling selectionsGood().  After selectionsGood()
            //       turn on the automagical clearing of the telemetry data which is the default action.

            telemetry.setAutoClear(false);
            telemetry.addLine().addData("Autonomous", "Selections");
            telemetry.addLine().addData("Alliance:", isRed ? "Red  " : "Blue  ").addData("  Robot Start Location:", startLocation ? "Crater" : "Depot");
            telemetry.addLine().addData("Deploy From Lander:", deployFromLander ? "Yes  " : "No  ").addData("  Claim Depot:", claimDepot ? "Yes" : "No");
            telemetry.addLine().addData("Park In Crater:", parkInCrater ? "Yes  " : "No  ");
            telemetry.update();

            // Verify that the autonomous selections are good, if so we are ready to rumble.  If not, well ask again.

            selectionsAreGood = myAO.selectionsGood();
            telemetry.setAutoClear(true);
            telemetry.update();    // Clear the selections
        }

        // Log autonomous selections
        L.Info("Initialized: Waiting for Start");
        L.Debug("isRed: ", isRed);
        L.Debug("startLocation: ", startLocation);
        L.Debug("deployFromLander: ", deployFromLander);
        L.Debug("claimDepot: ", claimDepot);
        L.Debug("parkInCrater: ", parkInCrater);

    //    robot.initialize(hardwareMap);

        gamepad1.reset();

        // Display autonomous status

        rtd.displayRobotTelemetry("Initialized Waiting for Start");
        rtd.displayRobotTelemetry("Alliance:",isRed ? "Red" : "Blue");
        rtd.displayRobotTelemetry("Start Location:",startLocation ? "Crater" : "Depot");
        rtd.displayRobotTelemetry("Deploy From Lander:",deployFromLander ? "Yes" : "No");
        rtd.displayRobotTelemetry("Park In Crater:",parkInCrater ? "Yes" : "No");


        // Wait for start to be pushed
        waitForStart();

        // Display autonomous status

        double encoderValue = 4567;
        double colorSensorValue = 5.0;
        double distanceSensorValue = 12.5;
        double heading = 45.78;

        rtd.displayRobotTelemetry("Deploying");
        rtd.displayRobotTelemetry("Encoder value:",String.valueOf(encoderValue));
        rtd.displayRobotTelemetry("Color Sensor Value:",String.valueOf(colorSensorValue));
        rtd.displayRobotTelemetry("Distance Sensor Value:",String.valueOf(distanceSensorValue));
        rtd.displayRobotTelemetry("Heading:",String.valueOf(heading));

        Thread.sleep(2000);

        rtd.displayRobotTelemetry("Sampling");
        rtd.displayRobotTelemetry("Encoder value:",String.valueOf(encoderValue));
        rtd.displayRobotTelemetry("Color Sensor Value:",String.valueOf(colorSensorValue));
        rtd.displayRobotTelemetry("Distance Sensor Value:",String.valueOf(distanceSensorValue));
        rtd.displayRobotTelemetry("Heading:",String.valueOf(heading));

        Thread.sleep(2000);

        rtd.displayRobotTelemetry("Parked - AutoComplete");
        rtd.displayRobotTelemetry("Encoder value:",String.valueOf(encoderValue));
        rtd.displayRobotTelemetry("Color Sensor Value:",String.valueOf(colorSensorValue));
        rtd.displayRobotTelemetry("Distance Sensor Value:",String.valueOf(distanceSensorValue));
        rtd.displayRobotTelemetry("Heading:",String.valueOf(heading));

        Thread.sleep(2000);




        /*
        // Deploy From Lander
        if (deployFromLander) {
            DeployRobot(robot);
        }

        // Move to Depot
      //  if (moveToDepot) {
            if (nearCrater) {
                moveDepotFromCraterStart(robot);
            }
            if (!nearCrater) {
                moveDepotFromDepotStart(robot);
            }

       // } */

    }

}