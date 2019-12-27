package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "Capture Servo Test")

public class CaptureServoTester extends LinearOpMode {

    public Servo stoneCaptureServo = null;
    public Servo stoneSwingServo = null;
    public Servo foundationGrabberLeft = null;
    public Servo foundationGrabberRight = null;

    public double servoPosition = 0.7;

    @Override
    public void runOpMode() {

        stoneCaptureServo = hardwareMap.get(Servo.class, "stoneCaptureServo");

        stoneCaptureServo.setPosition(servoPosition);

        telemetry.addData("Press Start to Scan the blasted Servos!", "");

        telemetry.addData("Servo Position", "%5.2f", servoPosition);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && servoPosition > 0.3) {

            stoneCaptureServo.setPosition(servoPosition);
            sleep(2000);

            telemetry.addData("Servo Position", "%5.2f", servoPosition);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            servoPosition -= 0.1;
        }

        sleep(5000);
    }

}
