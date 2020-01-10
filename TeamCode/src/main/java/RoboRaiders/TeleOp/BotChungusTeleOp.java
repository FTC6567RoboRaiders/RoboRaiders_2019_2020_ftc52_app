package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import RoboRaiders.Robot.Robot;

@TeleOp (name="BotChungus TeleOp")

public class BotChungusTeleOp extends OpMode {

    public Robot robot = new Robot();

    float backLeft;   // Power for left back motor
    float backRight;  // Power for right back motor
    float frontLeft;  // Power for left front motor
    float frontRight; // Power for right front motor
    float liftMotor;  // Power for lift

    public boolean currStateX = false;
    public boolean currStateY = false;
    public boolean currStateA = false;
    public boolean currStateB = false;
    public boolean currStateRightBumper = false;
    public boolean currStateLeftBumper = false;
    public double currStateRightTrigger = 0.0;
    public double currStateLeftTrigger = 0.0;
    public boolean currRightDPadState = false;
    public boolean currLeftDPadState = false;
    public boolean prevLeftDPadState = false;
    public double pushTimeDPadLeft = 0.0;
    public double dropTimeStamp = 0.0;
    public boolean releaseState = false;

    @Override
    public void init() { /*This is the initialization routine that the robot undergoes. */

        robot.initialize(hardwareMap);
                   //in order to negate the fact that the
        //motors are placed on the robot
        //to mirror each other.
        // telemetry.addData("Initialized", true);

        robot.setCapstoneElbowUp();  // Keep the elbow up after init
        telemetry.update();
    }

    @Override
    public void loop() {

        // Drive Train motor processing

        backLeft = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;    // These lines establish the joystick input values as
        backRight = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;   // the float variables "backLeft", "backRight", "frontLeft", and "frontRight", which
        frontLeft = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;   //correspond to the back left, back right, front left,
        frontRight = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;  // and front right wheels of the robot.

        backLeft = Range.clip(backLeft, -1, 1);     // These lines clip the extreme ends of the joystick input
        backRight = Range.clip(backRight, -1, 1);   // values in the resulting floats to avoid exceeding
        frontLeft = Range.clip(frontLeft, -1, 1);   // values accepted by the program.
        frontRight = Range.clip(frontRight, -1, 1);

        backLeft = (float) scaleInput(backLeft);        // These lines scale the joystick input values in the
        backRight = (float) scaleInput(backRight);      // resulting floats to the values in the array in the
        frontLeft = (float) scaleInput(frontLeft);      // double below, which are the only ones the program
        frontRight = (float) scaleInput(frontRight);    // accepts.

        robot.setDriveMotorPower(backLeft, backRight, frontLeft, frontRight);  // This line is an implementation of the public void
        // "setMotorPower" below. It sets the power of the motors to the joystick input values in
        // the floats.


        liftMotor = gamepad2.left_stick_y;
        robot.setLiftMotorPower(liftMotor);

        currStateX = gamepad2.x;
        currStateY = gamepad2.y;

        currStateA = gamepad2.a;
        currStateB = gamepad2.b;

        currStateLeftTrigger = gamepad2.left_trigger;
        currStateRightTrigger = gamepad2.right_trigger;

        currStateRightBumper = gamepad2.right_bumper;
        currStateLeftBumper = gamepad2 .left_bumper;

        currRightDPadState = gamepad2.dpad_right;
        currLeftDPadState = gamepad2.dpad_left;

    //Handles bringing the stone into the center of robot

      if (currStateLeftBumper){
          robot.setIntakePower(-1.0);
      }
      else if (currStateRightBumper){
          robot.setIntakePower(1.0);
      }
      else {
          robot.setIntakePower(0);
      }

        //Handles capturing the stone
        if (currStateX) {
            robot.setCaptureServoDown();
     //       robot.setCapstoneElbowUp();
        }
        else if (currStateY) {
            robot.setCaptureServoUp();
     //       robot.setCapstoneElbowInit();
        }

      //Handles the swing
      if (currStateA) {
          robot.setStoneSwingServoOut();
      }
      else if (currStateB) {
          robot.setStoneSwingServoIn();
      }

      //Handles grabbing the foundation
      if (currStateRightTrigger > 0.0 ) {
          robot.setFoundationGrabberGrabbed();
      }
      else if (currStateLeftTrigger > 0.0 ) {
          robot.setFoundationGrabberUnGrabbed();
      }

//      // The touch sensor is pressed AND the left dpad is not pressed AND the left dpad was not
//      // pressed previously AND the capstone was not released
//      if (robot.isLiftTouchSensorPressed() && !currLeftDPadState && !prevLeftDPadState && !releaseState) {
//          robot.setCapstoneElbowInit();
//
//      }

      // ELSE - the touch sensor is not pressed AND the left dpad is not pressed AND the left dpad
      // was not pressed previously
//      else if (!robot.isLiftTouchSensorPressed() && !currLeftDPadState && !prevLeftDPadState) {
//      else if (!robot.isLiftTouchSensorPressed()) {
//          robot.setCapstoneElbowUp();
//      }

      //capstone
      //was the left d pad pushed, and it wasn't pushed the last time through
      if (currLeftDPadState && !prevLeftDPadState) {
          prevLeftDPadState = true; //this was pushed the previous time
          pushTimeDPadLeft = System.currentTimeMillis(); //this is the time the d pad was pushed
          currLeftDPadState = false; //forget that it was currently pushed
          robot.setCapstoneElbowDown(); //position the elbow servo down
      }

      //was the button previously pushed, and has a quarter of a second expired?
      if (prevLeftDPadState && (System.currentTimeMillis() - pushTimeDPadLeft)>1000) {
          prevLeftDPadState = false; //we are done processing the d pad push
          robot.setCapstonePincherOpen(); //open the pincher servo
          pushTimeDPadLeft = 0.0; //reset the time
          dropTimeStamp = System.currentTimeMillis();
          releaseState = true;
      }

      if (releaseState && (System.currentTimeMillis() - dropTimeStamp)>1000) {
          robot.setCapstoneElbowUp();
          releaseState = false;
          dropTimeStamp = 0.0;
      }

        if(currRightDPadState){
            robot.setCapstoneElbowUp(); //set the elbow servo to up
            prevLeftDPadState = false; //stop left d pad processing
        }
    }

    @Override
    public void stop() {

    }


    double scaleInput(double dVal) {        // When implemented above, this double scales the joystick input values
        // in the floats.

        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}

