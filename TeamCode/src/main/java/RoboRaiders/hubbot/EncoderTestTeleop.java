package RoboRaiders.hubbot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HubbotServoTest", group = "Hubbot")
@Disabled
public class EncoderTestTeleop extends LinearOpMode {

  private DcMotor motor1;
  private DcMotor motor2;       // Motor objects


  public boolean currStateY = false;
  public boolean prevStateY = false;
  public boolean currStateX = false;
  public boolean prevStateX = false;
  public double servoPos = 0.0;

  @Override
  public void runOpMode() {

    // get a reference to our Light Sensor object.
    motor1 = hardwareMap.get(DcMotor.class, "motor1");

    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    motor2 = hardwareMap.get(DcMotor.class, "motor2");
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    // wait for the start button to be pressed.
    waitForStart();


    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      currStateX = gamepad1.x;
      currStateY = gamepad1.y;

      motor2.setPower(-gamepad1.left_stick_y);

      // send the info back to driver station using telemetry function.
      telemetry.addData("currStateX", currStateX);
      telemetry.addData("currStateY", currStateY);

      if (currStateX) {
        if (!motor1.isBusy()) {

          motor1.setTargetPosition(5000);
          motor1.setPower(0.5);
          telemetry.addData("motor1 RUN_TO_POSITION, 5000, power: 0.5, currentPosition: ", motor1.getCurrentPosition());
        }
//        else {
//          motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//          motor1.setPower(0.0);
//          telemetry.addData("motor1 RUN_WITHOUT_ENCODER, power: ", "0.0");
//        }
      }
      else if (currStateY) {
        motor1.setTargetPosition(0);
        motor1.setPower(-0.5);
        telemetry.addData("motor1 RUN_TO_POSITION, 0, power: -0.5, currentPosition: ", motor1.getCurrentPosition());
      }



      telemetry.update();
    }
  }
}
