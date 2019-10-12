package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HubbotServoTest", group = "Hubbot")
@Disabled
public class HubBotServoTest extends LinearOpMode {

  Servo contServo;  // Hardware Device Object

  public boolean currStateY = false;
  public boolean prevStateY = false;
  public boolean currStateX = false;
  public boolean prevStateX = false;
  public double servoPos = 0.0;

  @Override
  public void runOpMode() {

    // get a reference to our Light Sensor object.
    contServo = hardwareMap.get(Servo.class, "contServo");

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      currStateX = gamepad1.x;
      currStateY = gamepad1.y;

      // send the info back to driver station using telemetry function.
      telemetry.addData("currStateX", currStateX);
      telemetry.addData("currStateY", currStateY);

      if (currStateX) {
        servoPos = 1.0;
      }
      else if (currStateY) {
        servoPos = 0.0;
      }
      else {
        servoPos = 0.5;
      }
      telemetry.addData("servoPos","(%.2f)",servoPos);

      contServo.setPosition(servoPos);

      telemetry.update();
    }
  }
}
