package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "Capstone Pincher Test")

public class capstonePincherTest extends LinearOpMode{

    public Servo capstonePincher = null;
//    public Servo capstoneElbow = null;


    //public double servoPositionPincher = 1.0; //.35 IS GOOD FOR PINCHER CLOSED
    public double servoPositionElbow = 0.0; //0.3 IS INIT, 0.15 IS GOOD FOR THE STONE TO LIFT, 0.7 GOOD FOR DOWN
    public double servoPositionPincher = 0.35;

    @Override
    public void runOpMode() {

//        capstoneElbow = hardwareMap.get(Servo.class, "capstoneElbow");
        capstonePincher = hardwareMap.get(Servo.class, "capstonePincher");
        boolean done = false;

        telemetry.addData("Press Start to Scan the blasted Servos!", "");
        telemetry.update();

        capstonePincher.setPosition(servoPositionPincher);
//        capstoneElbow.setPosition(servoPositionElbow);
        telemetry.addData("Servo Pincher", "%5.2f", servoPositionPincher);
//        telemetry.addData("Servo Elbow", "%5.2f", servoPositionElbow);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !done) {

            capstonePincher.setPosition(servoPositionPincher);
            sleep(1500);

            telemetry.addData("Servo Position Pincher", "%5.2f", servoPositionPincher);
            telemetry.update();

            servoPositionPincher += 0.1;

        }
    }

}
