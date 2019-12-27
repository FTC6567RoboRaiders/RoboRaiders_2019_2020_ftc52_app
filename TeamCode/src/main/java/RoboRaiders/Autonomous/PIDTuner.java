package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.Robot.PidUdpReceiver;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous
@Disabled
public class PIDTuner extends RRAutonomousMethods {

   // public NostromoBotMotorDumper robot = new NostromoBotMotorDumper();


    private PidUdpReceiver pidUdpReceiver;
    private RobotTelemetryDisplay rtd;
    private double kP, kI, kD, degrees, direction;


    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        // Create new instance of robot telemetry display
        rtd = new RobotTelemetryDisplay(this,"JarJar");

        // Create new instance of receiver
        pidUdpReceiver = new PidUdpReceiver();

        // Start listening
        pidUdpReceiver.beginListening();

        // initialize the robot
     //   robot.initialize(hardwareMap);

        // set the transmission interval to 50 milliseconds
        telemetry.setMsTransmissionInterval(50);

        // Wait for start to be pushed
        waitForStart();

        while (opModeIsActive()) {

            updatePIDCoefficients();

            rtd.displayRobotTelemetry("PID Tuner");

            rtd.displayRobotTelemetry("kP", String.valueOf(kP));
            rtd.displayRobotTelemetry("ki", String.valueOf(kI));
            rtd.displayRobotTelemetry("kD", String.valueOf(kD));
            rtd.displayRobotTelemetry("Degrees", String.valueOf(degrees));

            if (direction == 0.0) {rtd.displayRobotTelemetry("Turn Direction","Right");}
            else {rtd.displayRobotTelemetry("Turn Direction","Left");}
        }

        pidUdpReceiver.shutdown();
    }

    public void updatePIDCoefficients() {

        kP = pidUdpReceiver.getP();
        kI = pidUdpReceiver.getI();
        kD = pidUdpReceiver.getD();
        degrees = pidUdpReceiver.getDegrees();
        direction = pidUdpReceiver.getDirection();
    }
}
