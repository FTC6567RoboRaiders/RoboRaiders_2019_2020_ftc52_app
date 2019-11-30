package RoboRaiders.AutonomousOptions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethods.JarJarAutonomousMethods;
import RoboRaiders.Robot.JarJarBot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous

public class AutonomousOptionsV1 extends JarJarAutonomousMethods {

    public boolean getSkystone = false;
    public boolean crossSkyBridge = false;
    public boolean selectionsAreGood = false;

    public JarJarBot robot = new JarJarBot();

    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        // Ask drivers how they want autonomous to work
        RoboRaiders.AutonomousOptions.AutoOptions myAO = new RoboRaiders.AutonomousOptions.AutoOptions(this);

        // Set up robot telemetry
        RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this, "JarJar");


        // While the drivers haven't made up their mind, keep asking what they want to do
        while (!selectionsAreGood) {

            getSkystone = myAO.selectGetSkystone();
            crossSkyBridge = myAO.crossSkyBridge();
            telemetry.setAutoClear(false);
            telemetry.addLine().addData("Autonomous", "Selections");
            telemetry.addLine().addData("get Skystone", getSkystone ? "Yes  " : "No  ");
            telemetry.addLine().addData("cross SkyBridge", crossSkyBridge ? "Yes " : "No ");
            telemetry.update();

            // Verify that the autonomous selections are good, if so we are ready to rumble.  If not, well ask again.

            selectionsAreGood = myAO.selectionsGood();
            telemetry.setAutoClear(true);
            telemetry.update();    // Clear the selections
        }

        gamepad1.reset();
        rtd.displayRobotTelemetry("Initialized Waiting for Start");
        rtd.displayRobotTelemetry("get Skystone",getSkystone ? "Yes" : "No");
        rtd.displayRobotTelemetry("cross SkyBridge ", crossSkyBridge ? "Yes " : "No ");
        waitForStart();

        telemetry.setAutoClear(true);
        telemetry.update();

        if(getSkystone){
            stoneSampling(robot);
        }

        if(crossSkyBridge){
            crossSkyBridge(robot);
        }
    }
}