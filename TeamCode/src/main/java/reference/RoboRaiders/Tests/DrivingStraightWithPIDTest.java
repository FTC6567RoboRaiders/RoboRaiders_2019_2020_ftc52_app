//package RoboRaiders.Tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID;
//import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
//import RoboRaiders.Robot.NostromoBotMotorDumper;
//import RoboRaiders.Robot.PidUdpReceiver;
//import RoboRaiders.Robot.RobotTelemetryDisplay;
//
//@Autonomous
//@Disabled
//public class DrivingStraightWithPIDTest extends NostromoAutonomousMethods {
//    public NostromoBotMotorDumper robot = new NostromoBotMotorDumper();
//
//
//    private PidUdpReceiver pidUdpReceiver;
//    private RoboRaidersPID rrPID;
//    private RobotTelemetryDisplay rtd;
//    private double kP, kI, kD, direction, degrees;
//
//
//    //----------------------------------------------------------------------------------------------
//    // Main logic
//    //----------------------------------------------------------------------------------------------
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // Create new instance of robot telemetry display
//        //rtd = new RobotTelemetryDisplay(this,"Nostromo");
//
//        // Create new instance of receiver
//        pidUdpReceiver = new PidUdpReceiver();
//
//        //Create new rrPID
//        rrPID = new RoboRaidersPID(0,0,0);
//
//        // Start listening
//        pidUdpReceiver.beginListening();
//
//        // initialize the robot
//        robot.initialize(hardwareMap);
//
//        // set the transmission interval to 50 milliseconds
//        telemetry.setMsTransmissionInterval(50);
//
//
//        // Wait for start to be pushed
//        waitForStart();
//
//        if (opModeIsActive()) {
//
//            updatePIDCoefficients();
//
//            rrPID.setCoeffecients(kP,kI,kD);
//
//            //EncoderDrivePID(rrPID, robot, degrees, direction);
//        }
//
//        pidUdpReceiver.shutdown();
//    }
//
//
//    public void updatePIDCoefficients() {
//
//        kP = pidUdpReceiver.getP();
//        kI = pidUdpReceiver.getI();
//        kD = pidUdpReceiver.getD();
//        direction = pidUdpReceiver.getDirection();
//        degrees = pidUdpReceiver.getDegrees();
//
//        //rtd.displayRobotTelemetry("kP",String.valueOf(kP));
//    }
//}
//
//
