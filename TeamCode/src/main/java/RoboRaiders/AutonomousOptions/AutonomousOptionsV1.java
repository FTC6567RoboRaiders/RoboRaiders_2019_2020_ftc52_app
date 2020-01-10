package RoboRaiders.AutonomousOptions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.Robot.Robot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous

public class AutonomousOptionsV1 extends RRAutonomousMethods {

    public boolean wait              = false;          // wait?
    public boolean allianceBlue      = false;          // Default Red Alliance
    public boolean loadSide          = false;          // Default Build side
    public boolean getSkystone       = false;          // Default to not get a skystone
    public boolean repoFoundation    = false;          // Default to not repositioning the foundation
    public boolean selectionsAreGood = false;          // Default that selections are not good

    public Robot robot = new Robot();

    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        // Ask drivers how they want autonomous to work
        RoboRaiders.AutonomousOptions.AutoOptions myAO = new RoboRaiders.AutonomousOptions.AutoOptions(this);

        // Set up robot telemetry
        RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this, "BotChungus");


        // While the drivers haven't made up their mind, keep asking what they want to do
        while (!selectionsAreGood) {

            wait = myAO.selectWait();
            allianceBlue = myAO.selectAlliance();
            loadSide = myAO.selectLoadSide();
            getSkystone = myAO.selectGetSkystone();
            repoFoundation = myAO.selectRepoFoundation();


            telemetry.setAutoClear(false);
            telemetry.addLine().addData("Autonomous", "Selections");
            telemetry.addLine().addData("Wait?: ", wait ? "Yes" : "No");
            telemetry.addLine().addData("Alliance: ", allianceBlue ? "Blue" : "Red ");
            telemetry.addLine().addData("Field Side: ", loadSide ? "Load  " : "Build  ");
            telemetry.addLine().addData("Get Skystone: ", getSkystone ? "Yes  " : "No  ");
            telemetry.addLine().addData("Move Foundation: ", repoFoundation ? "Yes  " : "No  ");

            telemetry.update();

            // Verify that the autonomous selections are good, if so we are ready to rumble.  If not, well ask again.

            selectionsAreGood = myAO.selectionsGood();
            telemetry.setAutoClear(true);
            telemetry.update();    // Clear the selections
        }

        gamepad1.reset();
        rtd.displayRobotTelemetry("Initialized Waiting for Start");
        rtd.displayRobotTelemetry("Wait? ", wait ? "Yes" : "No");
        rtd.displayRobotTelemetry("Alliance ",allianceBlue ? "Blue" : "Red");
        rtd.displayRobotTelemetry("Field Side",loadSide ? "Load" : "Build");
        rtd.displayRobotTelemetry("Get Skystone",getSkystone ? "Yes" : "No");
        rtd.displayRobotTelemetry("Move Foundation",repoFoundation ? "Yes" : "No");
        waitForStart();

        robot.setCapstoneElbowUp();

        telemetry.setAutoClear(true);
        telemetry.update();


        if (wait){                          //wait 5 seconds
            robotSleep(5000);
        }

        //determine alliance
        if(allianceBlue){
            //do blue stuff
            if(loadSide){
                //do load stuff
                if(getSkystone){
                    //get Skystone once this is done, the robot will be past the Skybridge on the build side
                    // halfway between the bridge and the wall however the stone is not released
                    stoneSamplingWebcamBlue(robot);
                    if (repoFoundation){
                        //foundation yes
                        loadSideFoundationBlue(robot);
                    }
                    else{
                        //foundation no
                        deliverStone(robot);
                    }
                    //park code
                    parkSkyBridgeBlue(robot);
                }
                else{
                    //don't get Skystone and park
                }
            }
            else{
                //do build stuff
                if(repoFoundation){
                    //foundation yes
                    blueFoundation(robot);
                }
                else{
                    //foundation no
                }
            }
//            robot.setCapstoneElbowInit();
        }

        else{
            //do red stuff
            if(loadSide){
                //do load stuff
                if(getSkystone){
                    //get Skystone once this is done, the robot will be past the Skybridge on the build side
                    //halfway between the bridge and the wall however the stone is not released
                    stoneSamplingWebcamRed(robot);
                    if (repoFoundation){
                        //foundation yes
                        loadSideFoundationRed(robot);
                    }
                    else{
                        //foundation no
                        deliverStone(robot);
                    }
                    //park code
                    parkSkyBridgeRed(robot);
                }
                else{
                    //don't get Skystone and park
                }
            }
            else{
                //do build stuff
                if(repoFoundation){
                    redFoundation(robot);
                    //foundation yes
                }
                else{
                    //foundation no
                }
            }
//            robot.setCapstoneElbowInit();
        }

        //robot.setCapstoneElbowInit();

    }
}