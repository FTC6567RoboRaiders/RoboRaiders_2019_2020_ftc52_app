package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name= "intakeTest")

public class intakeTest extends LinearOpMode {


    public DcMotor intakeMotorRight = null;
    public DcMotor intakeMotorLeft = null;

    public double servoPosition = 0.0;

    @Override
    public void runOpMode() {
        telemetry.update();

        intakeMotorLeft = hardwareMap.get(DcMotor.class, "intakeMotorLeft");
        intakeMotorRight = hardwareMap.get (DcMotor.class, "intakeMotorRight");

        intakeMotorRight.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        intakeMotorLeft.setPower(-.5);
        intakeMotorRight.setPower(-.5);
        while (opModeIsActive()){}
        intakeMotorRight.setPower(0.0);
        intakeMotorLeft.setPower(0.0);


    }
}
