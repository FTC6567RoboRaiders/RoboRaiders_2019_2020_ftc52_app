package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "Swing Servo Test")

public class SwingServoTester extends LinearOpMode {


    public Servo stoneSwingServo = null;

    public double servoPosition = 0.0;

    @Override
    public void runOpMode() {

        stoneSwingServo = hardwareMap.get(Servo.class, "stoneSwingServo");

        telemetry.addData("Press Start to Scan the blasted Servos!", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            stoneSwingServo.setPosition(servoPosition);
            sleep(2000);

            telemetry.addData("Servo Position", "%5.2f", servoPosition);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            servoPosition += 0.1;
        }
    }
}
