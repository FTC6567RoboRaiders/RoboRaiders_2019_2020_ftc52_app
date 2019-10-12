package RoboRaiders.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import RoboRaiders.Robot.NostromoBotMotorDumper;

/**
 *  Created by Steve Kocik
 */

@TeleOp(name="Teleop: Lets Drive")

public class NostromoDriveMotorForDumperDisabledLift extends OpMode {

    public NostromoBotMotorDumper robot = new NostromoBotMotorDumper();

    /* Define variables */
    float LeftBack;   // Power for left back motor
    float RightBack;  // Power for right back motor
    float LeftFront;  // Power for left front motor
    float RightFront; // Power for right front motor
    float maxpwr;     // Maximum power of the four motors
    float lander;
    float collection;
    double powerFactor = 1;
    public boolean currStateRightBumper1 = false;
    public boolean prevStateRightBumper1 = false;
    public boolean currStateLeftBumper1  = false;
    public boolean prevStateLeftBumper1  = false;
    public boolean currStateRightTrigger = false;
    public boolean prevStateRightTrigger = false;
    public boolean currStateLeftTrigger  = false;
    public boolean prevStateLeftTrigger  = false;
    public boolean currStateRightDpad = false;
    public boolean prevStateRightDpad = false;
    public boolean currStateLeftDpad = false;
    public boolean prevStateLeftDpad = false;
    int timesPivoted = 0;
    public boolean currStateY = false;
    public boolean prevStateY = false;
    public boolean currStateX = false;
    public boolean prevStateX = false;
    public boolean currStateB = false;
    public boolean prevStateB = false;
    public boolean currStateA = false;
    public boolean prevStateA = false;
    public boolean currState1B = false;
    public boolean prevState1B = false;
    public boolean currState1X = false;
    public boolean prevState1X = false;
    public boolean currState1Y = false;
    public boolean prevState1Y = false;
    public boolean currStateDPadUp = false;
    public boolean currStateDPadDown = false;
    public boolean gp1_currStateDPadRight = false;
    public boolean gp1_currStateDPadLeft = false;
    public boolean gp1_prevStateDPadRight = false;
    public boolean gp1_prevStateDPadLeft = false;
    public String intakeStatus = null;
    public String dumperStatus = null;
    public String collectionStatus = null;
    public String intakeDoorStatus = null;
    public String sliderStatus = null;
    public String LEDStatus = null;
    public double startTime;
    public double currentTime;

    int robotState;
    public final static int ALLIANCE_IS_RED = 1;
    public final static int ALLIANCE_IS_BLUE = 2;
    public final static int CLAW_OPEN = 3;
    public final static int CLAW_CLOSED = 4;
    public final static int X_BUTTON = 5;
    public final static int B_BUTTON = 6;
    public final static int Y_BUTTON = 8;
    public final static int LEDS_OFF = 9;





    @Override
    public void init() {

        robot.initialize(hardwareMap);

        telemetry.addData("Initialized", true);
        telemetry.update();

    }

    @Override
    public void start() {


        startTime = System.currentTimeMillis();


        //   robot.initializeServosTeleOp();
    }

    @Override
    public void loop() {


        currentTime = System.currentTimeMillis();


        // "Mecanum Drive" functionality
        LeftBack = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        RightBack = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        LeftFront = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        RightFront = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;



        maxpwr = findMaxPower(LeftBack, LeftFront, RightBack, RightFront);

        LeftBack = LeftBack / maxpwr;
        RightBack = RightBack / maxpwr;
        LeftFront = LeftFront / maxpwr;
        RightFront = RightFront / maxpwr;

        LeftBack = (float) scaleInput(LeftBack);
        RightBack = (float) scaleInput(RightBack);
        LeftFront = (float) scaleInput(LeftFront);
        RightFront = (float) scaleInput(RightFront);

        robot.setDriveMotorPower(LeftFront * 0.95,
                RightFront * 0.95,
                LeftBack * 0.95,
                RightBack * 0.95);

        /**
         * support dPad strafing on game pad 1
         */

        gp1_currStateDPadLeft = gamepad1.dpad_left;


        if (gp1_currStateDPadLeft && gp1_currStateDPadLeft != gp1_prevStateDPadLeft) {

            robot.setDriveMotorPower(1.0, -1.0, -1.0, 1.0);
            gp1_prevStateDPadLeft = gp1_currStateDPadLeft;

        }
        else if (!gp1_currStateDPadLeft && gp1_currStateDPadLeft != gp1_prevStateDPadLeft) {

            gp1_prevStateDPadLeft = gp1_currStateDPadLeft;
        }
        else if (gp1_currStateDPadLeft && gp1_prevStateDPadLeft){
            robot.setDriveMotorPower(1.0, -1.0, -1.0, 1.0);
        }

        gp1_currStateDPadRight = gamepad1.dpad_right;

        if (gp1_currStateDPadRight && gp1_currStateDPadRight != gp1_prevStateDPadRight) {

            robot.setDriveMotorPower(-1.0, 1.0, 1.0, -1.0);
            gp1_prevStateDPadRight = gp1_currStateDPadRight;

        }
        else if (!gp1_currStateDPadRight && gp1_currStateDPadRight != gp1_prevStateDPadRight) {

            gp1_prevStateDPadRight = gp1_currStateDPadRight;
        }
        else if (gp1_currStateDPadRight && gp1_prevStateDPadRight){
            robot.setDriveMotorPower(-1.0, 1.0, 1.0, -1.0);
        }




        // "Power Factor" functionality
        currStateLeftBumper1 = gamepad1.left_bumper;
        if (currStateLeftBumper1 && currStateLeftBumper1 != prevStateLeftBumper1) {

            powerFactor = 0.5;
            prevStateLeftBumper1 = currStateLeftBumper1;
        }
        else if (!currStateLeftBumper1 && currStateLeftBumper1 != prevStateLeftBumper1) {

            prevStateLeftBumper1 = currStateLeftBumper1;
        }
        currStateRightBumper1 = gamepad1.right_bumper;
        if (currStateRightBumper1 && currStateRightBumper1 != prevStateRightBumper1) {

            powerFactor = 1;
            prevStateRightBumper1 = currStateRightBumper1;
        }
        else if (!currStateRightBumper1 && currStateRightBumper1 != prevStateRightBumper1) {

            prevStateRightBumper1 = currStateRightBumper1;
        }

        // Limit lift motor functionality until the last 45 seconds
        if (currentTime - startTime > 75 * 1000) {

            // "Set Lift motor power" functionality
            lander = gamepad2.right_stick_y;
            lander = Range.clip(lander, -1, 1);
            //lander = (float) scaleInput(lander);
            if (lander >= 0.2) {
                robot.setLiftMotorPower(-0.95);
            } else if (lander <= -0.2) {
                robot.setLiftMotorPower(0.95);
            } else if (lander > -0.2 && lander < 0.2) {
                robot.setLiftMotorPower(0.00);
            }
        }


        // "Set collection extension" functionality
        collection = gamepad2.left_stick_y;
        collection = Range.clip(collection, -1, 1);
        //collection = (float) scaleInput(collection);

        if (collection >= 0.2 ){
            robot.setLiftIntakePower(0.75);
            //move collection up
            //
        }

        else if (collection <= -0.2){
            robot.setLiftIntakePower(-0.75);
            //move collection down
        }

        else if (collection > -0.2 && collection <0.2){
            robot.setLiftIntakePower(0.00);
            //collection doesn't move
        }//maybe switch this to the dpad...



        //put the motor on the stick (power) (like the lander)
//controls the servo door in the collection mechanism
     /*   currStateLeftBumper1 = gamepad2.left_bumper;
        currStateRightBumper1 = gamepad2.right_bumper;

        // send the info back to driver station using telemetry function.
        telemetry.addData("currStateLeftBumper", currStateLeftBumper1);
        telemetry.addData("currStateRightBumper", currStateRightBumper1);

        if (currStateLeftBumper1) {
            robot.intakeDoor.setPosition(robot.intakeDoorOpen);
            intakeDoorStatus = "Door open";
        }
        else if (currStateRightBumper1) {
            robot.intakeDoor.setPosition(robot.intakeDoorClosed);
            intakeDoorStatus = "door closed";
        }
        else {
            robot.intakeDoor.setPosition(0.5);
            intakeDoorStatus = "Stopped";
        }
        telemetry.addData("servoPos","(%.2s)",intakeDoorStatus);



        telemetry.update();*/

        currStateLeftBumper1 = gamepad2.left_bumper;

        if (currStateLeftBumper1 && currStateLeftBumper1 != prevStateLeftBumper1) {

            robot.intakeDoor.setPosition(robot.intakeDoorClosed);
            prevStateLeftBumper1 = currStateLeftBumper1;
            intakeDoorStatus = "Closed";
        }
        else if (!currStateLeftBumper1 && currStateLeftBumper1 != prevStateLeftBumper1) {

            prevStateLeftBumper1 = currStateLeftBumper1;
        }

        currStateRightBumper1 = gamepad2.right_bumper;

        if (currStateRightBumper1 && currStateRightBumper1 != prevStateRightBumper1) {

            robot.intakeDoor.setPosition(robot.intakeDoorOpen);
            prevStateRightBumper1 = currStateRightBumper1;
            intakeDoorStatus = "Open";
        }
        else if (!currStateRightBumper1 && currStateRightBumper1 != prevStateRightBumper1) {

            prevStateRightBumper1 = currStateRightBumper1;
        }

        telemetry.addData("ServosPos", intakeDoorStatus);

        telemetry.update();

        currStateRightDpad = gamepad2.dpad_right;

        if (currStateRightDpad && currStateRightDpad != prevStateRightDpad) {

            robot.dumpWrist.setPosition(robot.dumpWristDump);
            prevStateRightDpad = currStateRightDpad;

        }
        else if (!currStateRightDpad && currStateRightDpad != prevStateRightDpad) {

            prevStateRightDpad = currStateRightDpad;
        }

        currStateLeftDpad = gamepad2.dpad_left;
        if (currStateLeftDpad && currStateLeftDpad != prevStateLeftDpad) {

            robot.dumpWrist.setPosition(robot.dumpWristNotDump);
            prevStateLeftDpad = currStateLeftDpad;intakeDoorStatus = "Closed";
        }
        else if (!currStateLeftDpad && currStateLeftDpad != prevStateLeftDpad) {

            prevStateLeftDpad = currStateLeftDpad;
        }

        currStateDPadUp = gamepad2.dpad_up;
        currStateDPadDown = gamepad2.dpad_down;

        // send the info back to driver station using telemetry function.
        telemetry.addData("currStateDPadUp", currStateDPadUp);
        telemetry.addData("currStateDPadDown", currStateDPadDown);

        if (currStateDPadUp) {
            robot.setMotorDrawerSlide(-1.0);
            sliderStatus = "slider in";
        }
        else if (currStateDPadDown) {
            robot.setMotorDrawerSlide(1.0);
            sliderStatus = "slider out ";
        }
        else {
            robot.setMotorDrawerSlide(0.0);
            sliderStatus = "Stopped";
        }
        telemetry.addData("MotorStatus","(%.2s)",sliderStatus);



        telemetry.update();


        if (gamepad2.right_trigger > 0){
            currStateRightTrigger = true;
        }

        else {
            currStateRightTrigger = false;
        }

        if (currStateRightTrigger && currStateRightTrigger != prevStateRightTrigger) {

            robot.liftClaw.setPosition(robot.liftClawClosed);
            prevStateRightTrigger = currStateRightTrigger;
            robot.setBlinkinPattern(CLAW_CLOSED);
            robotState = CLAW_CLOSED;

            //robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
            //robot.blinkinLedDriver.setPattern(robot.pattern);
        }
        else if (!currStateRightTrigger && currStateRightTrigger != prevStateRightTrigger) {

            prevStateRightTrigger = currStateRightTrigger;

            }




        if (gamepad2.left_trigger > 0) {
            currStateLeftTrigger = true;
        }

        else {
            currStateLeftTrigger = false;
        }

        if (currStateLeftTrigger && currStateLeftTrigger != prevStateLeftTrigger) {

            robot.liftClaw.setPosition(robot.liftClawOpen);
            prevStateLeftTrigger = currStateLeftTrigger;


        }

         else if (!currStateLeftTrigger && currStateLeftTrigger != prevStateLeftTrigger) {

            prevStateLeftTrigger = currStateLeftTrigger;
            }


            //testing//
//controls the intake servo
        currStateX = gamepad2.x;
        currStateY = gamepad2.y;

        // send the info back to driver station using telemetry function.
        telemetry.addData("currStateX", currStateX);
        telemetry.addData("currStateY", currStateY);

        if (currStateX) {
            robot.intake.setPosition(robot.intakeIn);
            intakeStatus = "In";
        }
        else if (currStateY) {
            robot.intake.setPosition(robot.intakeOut);
            intakeStatus = "Out";
        }
        else {
            robot.intake.setPosition(0.5);
            intakeStatus = "Stopped";
        }
        telemetry.addData("servoPos","(%.2s)",intakeStatus);



        telemetry.update();

//contorols the servos on the lift arm
        currStateA = gamepad2.a;
        currStateB = gamepad2.b;

        // send the info back to driver station using telemetry function.
        telemetry.addData("currStateA", currStateA);
        telemetry.addData("currStateB", currStateB);

        if (currStateA) {
            //robot.dumperUp();
            robot.setMotorDumppPower(-0.65);  // was -0.6, then -0.7
            robot.dumpWrist.setPosition(robot.dumpWristDump);
            dumperStatus = "Up?";

        }
        else if (currStateB) {
            //robot.dumperDown();
            robot.setMotorDumppPower(0.5);
            robot.dumpWrist.setPosition(robot.dumpWristNotDump);
            dumperStatus = "Down?";
        }
        else {
            //robot.dumperStop();
            robot.setMotorDumppPower(0.0);
            dumperStatus = "Stopped";
        }
        telemetry.addData("MotorStatus","(%.2s)",dumperStatus);


        telemetry.update();

        currState1X = gamepad1.x;
        currState1B = gamepad1.b;
        currState1Y = gamepad1.y;

        // send the info back to driver station using telemetry function.
        telemetry.addData("currState1X", currState1X);
        telemetry.addData("currState1B", currState1B);
        telemetry.addData("currState1Y", currState1Y);

        if (currState1X) {
            //robot.dumperUp();
            //robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
            //robot.blinkinLedDriver.setPattern(robot.pattern);
            LEDStatus = "BLUE";
            robot.setBlinkinPattern(X_BUTTON);
            robotState = X_BUTTON;

        }
        else if (currState1B) {
            //robot.dumperDown();
            //robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
            //robot.blinkinLedDriver.setPattern(robot.pattern);
            LEDStatus = "RED";
            robot.setBlinkinPattern(B_BUTTON);
            robotState = B_BUTTON;
        }
        else if (currState1Y) {
            //robot.dumperDown();
            //robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
            //robot.blinkinLedDriver.setPattern(robot.pattern);
            robot.setBlinkinPattern(Y_BUTTON);
            robotState = Y_BUTTON;
            //LEDStatus = "HANG";
        }
        else {

            LEDStatus = "OFF";
        }
        telemetry.addData("LED Status","(%.2s)", LEDStatus);


        telemetry.update();




/*
        currStateX = gamepad2.x;
        if (currStateX && currStateX != prevStateX) {

            robot.intake.setPosition(robot.intakeIn);
            prevStateX = currStateX;
            currStateX = false;
        }

        currStateY = gamepad2.y;
        if (currStateY && currStateY != prevStateY) {

            robot.intake.setPosition(robot.intakeOut);
            prevStateY = currStateY;
            currStateY = false;
        }
*/

        }



    @Override
    public void stop() {

    }

    /**
     * scaleInput will attempt to smooth or scale joystick input when driving the
     * robot in teleop mode.  By smoothing the joystick input more controlled movement
     * of the robot will occur, especially at lower speeds.
     * <br><br>
     * To scale the input, 16 values are used that increase in magnitude, the algorithm
     * will determine where the input value roughly falls in the array by multiplying it
     * by 16, then will use the corresponding array entry from the scaleArray variable to
     * return a scaled value.
     * <br><br>
     * <b>Example 1:</b> dVal (the input value or value passed to this method) is set to 0.76
     * <br>
     * Stepping through the algorithm
     * <ol>
     * <li> 0.76*16 = 12.16, but because we cast the calculations as an integer (int)
     * we lose the .16 so the value just is 12, variable index now contains 12.  <b>Note:</b>
     * the index variable will tell us which of the array entries in the scaleArray array to
     * use.</li>
     * <li> Check if the index is negative (less than zero), in this example the
     * variable index contains a positive 12</li>
     * <li> Check if the variable index is greater than 16, this is done so the
     * algorithm does not exceed the number of entries in the scaleArray array</li>
     * <li> Initialize the variable dScale to 0.0 (not really needed but we are
     * just being safe)</li>
     * <li> If dVal (value passed to this method) was initially negative, then
     * set the variable dScale to the negative of the scaleArray(index), in this example
     * dVal was initially 0.76 so not negative</li>
     * <li> If dVal (value passed to this method) was initially positive, then
     * set the variable dScale to the scaleArray(index), since index is 12, then
     * scaleArray(12) = 0.60.  <b>Remember, in java the first array index is 0,
     * this is why scaleArray(12) is not 0.50</b></li>
     * <li> Return the dScale value (0.60)</li>
     * </ol>
     * <p>
     * <br><br>
     * <b>Example 2</b> dVal (the input value or value passed to this method) is set to -0.43
     * <br>
     * Stepping through the algorithm
     * <ol>
     * <li> -0.43*16 = -6.88, but because we cast the calculations as an integer (int)
     * we lose the .88 so the value just is -6, variable index now contains -6.  <b>Note:</b>
     * the index variable will tell us which of the array entries in the scaleArray array to
     * use.</li>
     * <li> Check if the index is negative (less than zero), in this example the
     * variable index is negative, so make the negative a negative (essentially
     * multiplying the variable index by -1, the variable index now contains 6</li>
     * <li> Check if the variable index is greater than 16, this is done so the
     * algorithm does not exceed the number of entries in the scaleArray array</li>
     * <li> Initialize the variable dScale to 0.0 (not really needed but we are
     * just being safe)</li>
     * <li> If dVal (value passed to this method) was initially negative, then
     * set the variable dScale to the negative of the scaleArray(index), in this example
     * dVal was initially -0.43, so make sure to return a negative value of scaleArray(6).
     * scaleArray(6) is equal to 0.18 and the negative of that is -0.18 <b>Remember,
     * in java the first array index is 0, this is why scaleArray(6) is not 0.15</b></li>
     * <li> Return the dScale value (-0.18)</li>
     * </ol>
     *
     * @param dVal the value to be scaled -between -1.0 and 1.0
     * @return the scaled value
     * <B>Author(s)</B> Unknown - copied from internet
     */
    double scaleInput(double dVal) {
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

    /**
     * findMaxPower - finds the maximum power of four power values
     *
     * @param pwr1 first power
     * @param pwr2 second power
     * @param pwr3 third power
     * @param pwr4 fourth power
     * @return maximum power of the four values
     * <B>Author(s):</B> Jason Sember and Steeeve
     */
    float findMaxPower(float pwr1, float pwr2, float pwr3, float pwr4) {

        float maxpwrA = Math.max(Math.abs(pwr1), Math.abs(pwr2));
        float maxpwrB = Math.max(Math.abs(pwr3), Math.abs(pwr4));
        float maxpwr = Math.max(Math.abs(maxpwrA), Math.abs(maxpwrB));

        if (maxpwr > 1.0) {

            return maxpwr;
        } else {

            return 1;
        }


    }


}