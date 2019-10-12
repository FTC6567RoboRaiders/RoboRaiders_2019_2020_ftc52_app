package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import RoboRaiders.AutonomousMethods.AutoOptions.AutoOptions;
import RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID;
import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Logger.LoggerOId;
import RoboRaiders.Robot.NostromoBotMotorDumper;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous
public class NostromoAutonomousoptionsV3 extends NostromoAutonomousMethods{


    public final static int ALLIANCE_IS_RED = 1;
    public final static int ALLIANCE_IS_BLUE = 2;
    public final static int CLAW_OPEN = 3;
    public final static int CLAW_CLOSED = 4;
    public final static int X_BUTTON = 5;
    public final static int B_BUTTON = 6;
    public final static int Y_BUTTON = 8;
    public final static int LEDS_OFF = 9;

    public boolean isRed             = false;
    private boolean startLocation     = false;
    private boolean deployFromLander  = false;
    private boolean claimDepot        = false;
    private boolean parkInCrater      = false;
    private boolean selectionsAreGood = false;
    private boolean sampling     = false;
    private boolean sampleWait       = false;
    public NostromoBotMotorDumper robot = new NostromoBotMotorDumper();
    //public RoboRaidersPID turningPID = new RoboRaidersPID(0.02,0.0,0.1);
    public RoboRaidersPID turningPID = new RoboRaidersPID(0.024,0.0,0.02);
    public RoboRaidersPID drivePID = new RoboRaidersPID(0.002,0.0,0.005);



    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        // Ask drivers how they want autonomous to work
        AutoOptions myAO = new AutoOptions(this);

        // Set up for logging messages to the log
        LoggerOId L = new LoggerOId(String.valueOf("FTC6567"));

        // Set up robot telemetry
        RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"Nostromo");


        // While the drivers haven't made up their mind, keep asking what they want to do
        while (!selectionsAreGood) {

            isRed            = myAO.selectAlliance();              // Get the alliance (Red or Blue)
            startLocation    = myAO.selectStartLocation();         // Get where the robot is starting from (Depot or Crater)
            deployFromLander = myAO.selectDeployFromLander();      // Should the robot deploy from the lander (Yes or No)
            sampling    = myAO.selectSampling();              // Should the robot sample for minerals (Yes or No)
            sampleWait      = myAO.selectWait();                   //Should we wait to sample?
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
            telemetry.addLine().addData("Deploy From Lander:", deployFromLander ? "Yes  " : "No  ").addData("  Sample Mineral: ", sampling ? "Depot" : "Crater");
            telemetry.addLine().addData("  Claim Depot:", claimDepot ? "Yes" : "No").addData("Park In Crater:", parkInCrater ? "Yes  " : "No  ");
            telemetry.addLine().addData("Wait for Sampling", sampleWait ? "Yes " : "No ");
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
        L.Debug("sampleWait:", sampleWait);
        L.Debug("claimDepot: ", claimDepot);
        L.Debug("parkInCrater: ", parkInCrater);

        //    robot.initialize(hardwareMap);

        gamepad1.reset();

        // Display autonomous status

        rtd.displayRobotTelemetry("Initialized Waiting for Start");
        rtd.displayRobotTelemetry("Alliance:",isRed ? "Red" : "Blue");
        rtd.displayRobotTelemetry("Start Location:",startLocation ? "Crater" : "Depot");
        rtd.displayRobotTelemetry("Deploy From Lander:",deployFromLander ? "Yes" : "No");
        rtd.displayRobotTelemetry("Wait to Sample:",sampleWait ? "Yes" : "No");
        rtd.displayRobotTelemetry("Park In Crater:",parkInCrater ? "Yes" : "No");

        if (sampling == true) {
            robot.initVuforia();

            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                robot.initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
            /*if (robot.tfod != null) {
                robot.tfod.activate();
            }*/

        }


        // Wait for start to be pushed.
        waitForStart();

        telemetry.setAutoClear(true);
        telemetry.update();
        // Deploy From Lander
        if (deployFromLander) {
            DeployRobot(robot);
        }

        if (sampling){//asking if sampling
            if (startLocation){//are we starting from the crater?
                samplingMineralsCrater(drivePID, turningPID, robot, sampleWait);
            }
            else {
                samplingMineralsDepot(drivePID, turningPID, robot, sampleWait);
            }
        }
        else{
            if (startLocation){
                mineralCenterCrater(drivePID, turningPID, robot);
            }
            else{
                mineralCenterDepot(drivePID, turningPID, robot);
            }
        }



        // Is the robot starting facing the crater
        /*if (startLocation) {
            moveDepotFromCraterStart(robot);
        }
        // Is the robot starting facing the crater
        else{


            moveDepotFromDepotStart(robot);
        }*/
        // Are we claiming the depot
        if (claimDepot) {
            DeployTeamMarker(drivePID, turningPID, robot, startLocation);
        }
        // Ending Parked in crater
        if (parkInCrater) {
            if (startLocation){ parkFromCraterStart(drivePID, turningPID, robot); } // started crater side
            else { parkFromDepotStart(drivePID, turningPID, robot); } // started depot side
        }

    }
}
