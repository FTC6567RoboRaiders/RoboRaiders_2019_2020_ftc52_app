package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "Foundation Servo Test")

public class FoundationServoTester extends LinearOpMode {

    public Servo foundationGrabberLeft = null;
    public Servo foundationGrabberRight = null;

    public double servoPositionRight = 0.35;
    public double servoPositionLeft = 1.0;

    @Override
    public void runOpMode() {

        foundationGrabberLeft = hardwareMap.get(Servo.class, "foundationGrabberLeft");
        foundationGrabberRight = hardwareMap.get(Servo.class, "foundationGrabberRight");

        telemetry.addData("Press Start to Scan the blasted Servos!", "");
        telemetry.update();

        foundationGrabberRight.setPosition(servoPositionRight);
        foundationGrabberLeft.setPosition(servoPositionLeft);
        telemetry.addData("Servo Position Right", "%5.2f", servoPositionRight);
        telemetry.addData("Servo Position Left", "%5.2f", servoPositionLeft);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            foundationGrabberRight.setPosition(servoPositionRight);
            foundationGrabberLeft.setPosition(servoPositionLeft);
            sleep(1500);

            telemetry.addData("Servo Position Right", "%5.2f", servoPositionRight);
            telemetry.addData("Servo Position Left", "%5.2f", servoPositionLeft);
            telemetry.update();

            servoPositionRight += 0.1;
            servoPositionLeft -= 0.1;


        }
    }

}
