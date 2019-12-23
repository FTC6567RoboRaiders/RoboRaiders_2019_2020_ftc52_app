package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "Servo Test")

public abstract class SwingServoTester extends LinearOpMode {

    public Servo stoneCaptureServo = null;
    public Servo stoneSwingServo = null;
    public Servo foundationGrabberLeft = null;
    public Servo foundationGrabberRight = null;

    public double servoPosition = 0.0;

    public void runOpMode() {

        stoneSwingServo = hardwareMap.get(Servo.class, "stoneSwingServo");

        telemetry.addData("Press Start to Scan the blasted Servos!", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            stoneSwingServo.setPosition(servoPosition);
            sleep(500);

            telemetry.addData("Servo Position", "%5.2f", servoPosition);
            telemetry.addData(">", "Press Stop to end test." );

            servoPosition += 0.1;


        }
    }
}
