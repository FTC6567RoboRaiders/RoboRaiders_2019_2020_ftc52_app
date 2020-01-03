package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "Capstone Servos Test")

public class capstoneServosTester extends LinearOpMode {

    public Servo capstonePincher = null;
    public Servo capstoneElbow = null;

    public double servoPositionPincher = 0.5;
    public double servoPositionElbow = 0.5;

    @Override
    public void runOpMode() {

        capstoneElbow = hardwareMap.get(Servo.class, "capstoneElbow");
        capstonePincher = hardwareMap.get(Servo.class, "capstonePincher");

        telemetry.addData("Press Start to Scan the blasted Servos!", "");
        telemetry.update();

        capstonePincher.setPosition(servoPositionPincher);
        capstoneElbow.setPosition(servoPositionElbow);
        telemetry.addData("Servo Pincher", "%5.2f", servoPositionPincher);
        telemetry.addData("Servo Elbow", "%5.2f", servoPositionElbow);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            capstonePincher.setPosition(servoPositionPincher);
            capstoneElbow.setPosition(servoPositionElbow);
            sleep(1500);

            telemetry.addData("Servo Position Right", "%5.2f", servoPositionPincher);
            telemetry.addData("Servo Position Left", "%5.2f", servoPositionElbow);
            telemetry.update();

            servoPositionPincher += 0.1;
            servoPositionElbow += 0.1;


        }
    }

}
