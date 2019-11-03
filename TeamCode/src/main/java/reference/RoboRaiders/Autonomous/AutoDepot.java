//package RoboRaiders.Autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID;
//import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
//import RoboRaiders.Robot.NostromoBotMotorDumper;
//
//@Disabled
//@Autonomous(name="Auto: Robot Starts Towards Depot")
//
//public class AutoDepot extends NostromoAutonomousMethods {
//
//    NostromoBotMotorDumper robot = new NostromoBotMotorDumper();
//    RoboRaidersPID robotPID = new RoboRaidersPID(0,0,0);
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.initialize(hardwareMap);
//
//        waitForStart();
//
//       // DeployRobot(robot);
//
//        //farRedDepot(robotPID, robot);
//
//        //moveTest(robotPID, robot);
//
//    }
//
//    }
//
