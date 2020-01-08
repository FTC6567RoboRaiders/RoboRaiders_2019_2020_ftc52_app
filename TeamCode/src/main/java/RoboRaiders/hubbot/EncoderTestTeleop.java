package RoboRaiders.hubbot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hubbot Encoder Teleop", group = "Hubbot")
@Disabled
public class EncoderTestTeleop extends LinearOpMode {

  private DcMotor motor1;
  private DcMotor motor2;       // Motor objects


  public boolean currStateY = false;      // current state of the gamepad1 Y button
  public boolean currStateX = false;      // current state of the gamepad1 X button
  public boolean rightsticky = false;     // determines if right stick y should be used for input


  @Override
  public void runOpMode() {

    // get a reference to motor1 from the hardware map, reset the encoders and set the zero power behaviour
    motor1 = hardwareMap.get(DcMotor.class, "motor1");
    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // get a reference to motor2 from the hardware map, reset encoders and run without encoder s
    motor2 = hardwareMap.get(DcMotor.class, "motor2");
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    // wait for the start button to be pressed.
    waitForStart();


    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      // save the state of the gamepad 1 X and Y buttons
      currStateX = gamepad1.x;
      currStateY = gamepad1.y;

      // motor 2 is only controlled by the left stick Y direction
      motor2.setPower(-gamepad1.left_stick_y);

      // send the info back to driver station using telemetry function.
      telemetry.addData("currStateX", currStateX);
      telemetry.addData("currStateY", currStateY);

      // When the X button is pressed on gamepad 1, motor1 will run to encoder position 5000 at 50% power
      if (currStateX) {
          rightsticky = false;                        // indicate that gamepad1 right stick input is to be ignored
          motor1.setTargetPosition(5000);
          motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motor1.setPower(0.5);
          telemetry.addData("motor1 RUN_TO_POSITION, 5000, power: 0.5, currentPosition: ", motor1.getCurrentPosition());
      }

      // When the Y button is pressed on gamepad 1, motor1 will run to encoder position 0 at 50% power
      else if (currStateY) {
        rightsticky = false;                          // indicate that gamepad1 right stick input is to be ignored
        motor1.setTargetPosition(0);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(-0.5);
        telemetry.addData("motor1 RUN_TO_POSITION, 0, power: -0.5, currentPosition: ", motor1.getCurrentPosition());
      }


      // if the gamepad1 right stick has been pushed (that is it is not zero), then indicate that the gamepad1 right stick
      // input is valid and should be applied to the power for motor1
      if (gamepad1.right_stick_y != 0.0) {
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightsticky = true;
      }

      // when gamepad1 right stick input is valid apply a power motor1
      if (rightsticky) {
        motor1.setPower(-gamepad1.right_stick_y);
      }

      telemetry.addData("motor1 RUN_TO_POSITION, currentPosition: ", motor1.getCurrentPosition());
      telemetry.update();
    }
  }
}
