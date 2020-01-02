package RoboRaiders.Robot;

import android.graphics.Color;
import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class Robot {



    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor intakeMotorRight = null;
    public DcMotor intakeMotorLeft = null;
    public DcMotor liftMotor = null;

    public Servo stoneCaptureServo = null;
    public Servo stoneSwingServo = null;
    public Servo foundationGrabberLeft = null;
    public Servo foundationGrabberRight = null;
    public Servo capstonePincher = null;
    public Servo capstoneElbow = null;

    public TouchSensor liftTouchSensor = null;

    public BNO055IMU imu;

    public DistanceSensor stoneDistanceSensor = null;
    public ColorSensor colorSensor = null;


    /* Local OpMode Members */
    public HardwareMap hwMap = null;

    /* Public Variables */
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;

    //Robot Constants
    private static final double CAPTURE_SERVO_UP = 0.7;
    private static final double CAPTURE_SERVO_DOWN = 0.3;
    private static final double SWING_SERVO_OUT = 1.0;                       //* from testing with SwingServoTester.java
    private static final double SWING_SERVO_IN = 0.0;                         //* from testing with SwingServoTester.java
    private static final double FOUNDATION_SERVO_GRAB_LEFT = 0.45;
    private static final double FOUNDATION_SERVO_GRAB_RIGHT = 0.95;
    private static final double FOUNDATION_SERVO_RELEASE_LEFT = 1.0;
    private static final double FOUNDATION_SERVO_RELEASE_RIGHT = 0.35;
    private static final int    COLOR_SENSOR_SCALE_FACTOR = 255;             // Scale factor used for color sensor
    private static final double CAPSTONE_PINCHER_CLOSED = 1.0;
    private static final double CAPSTONE_PINCHER_OPEN = 0.0;
    private static final double CAPSTONE_ELBOW_UP = 1.0;
    private static final double CAPSTONE_ELBOW_DOWN = 0.0;

    //public ModernRoboticsI2cRangeSensor distance;
    //public double takeSkystoneUp = 0.0;
    //public double takeSkystoneDown = 1.0;


    /**
     * Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     */
    public Robot() {

    }

    /**
     * This method will initialize the robot
     *
     * @param ahwMap hardware map for the robot
     */
    public void initialize(HardwareMap ahwMap) {

        // Save reference to hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        motorFrontLeft = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hwMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hwMap.get(DcMotor.class, "motorBackRight");
        intakeMotorLeft = hwMap.get(DcMotor.class, "intakeMotorLeft");
        intakeMotorRight = hwMap.get (DcMotor.class, "intakeMotorRight");
        liftMotor = hwMap.get (DcMotor.class, "liftMotor");


        // Defines the directions the motors will spin
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorRight.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //have the motors on the drivetrain break here.
        // Set all motors to zero power
        motorFrontRight.setPower(0.0);
        motorFrontLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        intakeMotorLeft.setPower(0.0);
        intakeMotorRight.setPower(0.0);
        liftMotor.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODER if encoders are installed, and we wouldn't use encoders for teleop, even if we
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //will use them in teleop.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

        //Define and initialize Servos
        stoneCaptureServo = hwMap.servo.get("stoneCaptureServo");
        stoneSwingServo = hwMap.servo.get("stoneSwingServo");
        foundationGrabberLeft = hwMap.servo.get("foundationGrabberLeft");
        foundationGrabberRight = hwMap.servo.get("foundationGrabberRight");
        capstoneElbow = hwMap.servo.get("capstoneElbow");
        capstonePincher = hwMap.servo.get("capstonePincher");

        setFoundationGrabberUnGrabbed();
        setCaptureServoUp();
        setStoneSwingServoIn();


        // Define sensors
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        stoneDistanceSensor = hwMap.get(DistanceSensor.class, "stoneDistanceSensor");
        liftTouchSensor = hwMap.touchSensor.get("liftTouchSensor");

    }



    //**********************************************************************************************
    //
    // DRIVE TRAIN METHODS
    //
    //**********************************************************************************************

    /**
     * This method will set the power for the drive motors
     *
     * @param leftFront  power setting for the left front motor
     * @param rightFront power setting for the right front motor
     * @param leftBack   power setting for the left back motor
     * @param rightBack  power setting for the right back motor
     */
    public void setDriveMotorPower(double leftFront, double rightFront, double leftBack, double rightBack) {

        motorFrontLeft.setPower(leftFront);
        motorBackRight.setPower(rightBack);
        motorBackLeft.setPower(leftBack);
        motorFrontRight.setPower(rightFront);

    }


    /**
     * Calculates the number of encoder counts to travel a given distance for the drive train motors
     * @param distance
     * @return
     */
    public double driveTrainCalculateCounts(double distance) {

        double COUNTS;

        int DIAMETER = 4; //diameter of wheel
        double GEAR_RATIO = (1.0 / 1.0); //gear ratio
        double PULSES = 537.6; //encoder counts in one revolution - neverest 20 orbital
        double CIRCUMFERENCE = Math.PI * DIAMETER; //gives you circumference
        double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; //gives the rotations
        COUNTS = PULSES * ROTATIONS; //gives the counts

        return COUNTS;
    }

    /**
     * Takes the four drive train encoder values and sorts them using a bubble sort algorithm from
     * lowest to highest.  Throws out the lowest and highest values in the sorted list and averages
     * the two remaining values
     * @return average of the two middle encoder counts
     */
    public int getSortedEncoderCount() {

        int[] encoderArray = new int[4];

        encoderArray[0] = Math.abs(motorFrontLeft.getCurrentPosition());
        encoderArray[1] = Math.abs(motorFrontRight.getCurrentPosition());
        encoderArray[2] = Math.abs(motorBackLeft.getCurrentPosition());
        encoderArray[3] = Math.abs(motorBackRight.getCurrentPosition());

        int I;
        int J;
        int Temp;

        for (I = 0; I < 3; I++) {
            for (J = I + 1; J < 4; J++) {
                if (encoderArray[I] < encoderArray[J]) {
                }
                else {

                    Temp = encoderArray[I];
                    encoderArray[I] = encoderArray[J];
                    encoderArray[J] = Temp;
                }
            }
        }
        int averageCount = (encoderArray[1] + encoderArray[2]) / 2;

        return averageCount;
    }

    /**
     * Sets the target encoder value for the drive train motors
     * @param encoderPosition
     */
    public void setDTMotorTargetPosition(int encoderPosition){
        motorFrontLeft.setTargetPosition(encoderPosition);
        motorFrontRight.setTargetPosition(encoderPosition);
        motorBackLeft.setTargetPosition(encoderPosition);
        motorBackRight.setTargetPosition(encoderPosition);
    }

     /**
     * This method will set the mode of all of the drive train motors to run using encoder
     */
    public void runWithEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * This method will set the mode all of the drive train motors to RUN_TO_POSITION
     */
    public void runWithEncodersSTP() {
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * This method will set the mode of all of the drive train motors to run without encoder
     */
    public void runWithoutEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * This will set the mode of the drive train motors to STOP_AND_RESET_ENCODER, which will zero
     * the encoder count but also set the motors into a RUN_WITHOUT_ENCODER mode
     */
    public void resetEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * These methods will get individual encoder position from any of the drive train motors
     * @return the encoder position
     */
    public double getBackLeftDriveEncoderCounts() { return motorBackLeft.getCurrentPosition(); }
    public double getBackRightDriveEncoderCounts() { return motorBackRight.getCurrentPosition(); }
    public double getFrontLeftDriveEncoderCounts() { return motorFrontLeft.getCurrentPosition(); }
    public double getFrontRightDriveEncoderCounts() { return motorFrontRight.getCurrentPosition(); }


    //**********************************************************************************************
    //
    // END DRIVE TRAIN METHODS
    //
    //**********************************************************************************************



    //**********************************************************************************************
    //
    // IMU METHODS
    //
    //**********************************************************************************************


    /**
     * Gets the current heading from the IMU
     * @return current heading in degrees
     */
    public float getHeading() {

        float heading;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
        heading = Math.abs(angles.firstAngle); //heading is equal to the absolute value of the first angle

        return heading;
    }


    /**
     * Re-initializes the IMU
     */
    public void resetIMU() {

        imu.initialize(parameters);
    }

    /**
     * Calculates and returns an integrate Z-Axis (aka heading).  This handles when the heading crosses
     * 180 or -180
     * @return integrated Z-Axis
     */
    public double getIntegratedZAxis() {

        // This sets up the how we want the IMU to report data
        iza_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Obtain the heading (Z-Axis)
        iza_newHeading = iza_angles.firstAngle;

        // Calculate the change in the heading from the previous heading
        iza_deltaHeading = iza_newHeading - iza_lastHeading;

        // Bosch IMU wraps at 180, so compensate for this
        if (iza_deltaHeading <= -180.0) {

            iza_deltaHeading += 360.0;
        }
        else if (iza_deltaHeading >= 180.0) {

            iza_deltaHeading -= 360;
        }

        // Calculate the integratedZAxis
        integratedZAxis += iza_deltaHeading;

        // Save the current heading for the next call to this method
        iza_lastHeading = iza_newHeading;

        return integratedZAxis;
    }

    //**********************************************************************************************
    //
    // END IMU METHODS
    //
    //**********************************************************************************************


    //**********************************************************************************************
    //
    // INTAKE METHODS
    //
    //**********************************************************************************************

    /**
     * Sets the intake power for the intake motors
     * @param intake - power to set the intake motors to
     */
    public void setIntakePower(double intake){
        intakeMotorLeft.setPower(intake);
        intakeMotorRight.setPower(intake);
    }

    //**********************************************************************************************
    //
    // END INTAKE METHODS
    //
    //**********************************************************************************************


    //**********************************************************************************************
    //
    // SERVO METHODS
    //
    //**********************************************************************************************


    /**
     * Sets the capture servo position to the passed in position
     * @param position the position the capture servo is to rotate to
     */
    public void setCaptureServoPosition (double position) {

        stoneCaptureServo.setPosition(position);
    }

    /**
     * Sets the capture servo position to the up position
     */
    public void setCaptureServoUp () {

        setCaptureServoPosition(CAPTURE_SERVO_UP); //Captures the stone
    }

    /**
     * Sets the capture servo position to the down position
     */
    public void setCaptureServoDown (){

        setCaptureServoPosition(CAPTURE_SERVO_DOWN);
    }

    /**
     * Sets the stone swing servo position
     * @param position the position the stone swing servo is to rotate to
     */
    public void setStoneSwingPosition (double position) {

        stoneSwingServo.setPosition(position);
    }

    /**
     * Sets the stone swing servo position to out (outside of the robot's footprint)
     */
    public void setStoneSwingServoOut () {

        setStoneSwingPosition(SWING_SERVO_OUT);
    }

    /**
     * Sets the stone swing servo position to in (inside of the robot's footprint)
     */
    public void setStoneSwingServoIn () {

        setStoneSwingPosition(SWING_SERVO_IN);
    }

    /**
     * Sets the foundation grabber servos position
     * @param posLeft - position of the left foundation grabber servo
     * @param posRight - position of the right foundation grabber servo
     */
    public void setfoundationGrabberPostion (double posLeft, double posRight) {
        foundationGrabberLeft.setPosition(posLeft);
        foundationGrabberRight.setPosition(posRight);
    }

    /**
     * Sets the foundation grabber servos to the grabbed or down position
     */
    public void setFoundationGrabberGrabbed () {
        setfoundationGrabberPostion(FOUNDATION_SERVO_GRAB_LEFT, FOUNDATION_SERVO_GRAB_RIGHT);

    }

    /**
     * Sets the foundation grabber servos to the ungrabbed or up position
     */
    public void setFoundationGrabberUnGrabbed () {
        setfoundationGrabberPostion(FOUNDATION_SERVO_RELEASE_LEFT, FOUNDATION_SERVO_RELEASE_RIGHT);
    }

    /**
     * Sets the capstone pincher servo position
     * @param position - the position of the capstone pincher servo
     */
    public void setCapstonePincherPosition (double position) {
        capstonePincher.setPosition(position);
    }

    /**
     * Sets the capstone pincher servo position to open
     */
    public void setCapstonePincherOpen () {
        setCapstonePincherPosition(CAPSTONE_PINCHER_OPEN);
    }

    /**
     * Sets the capstone pincher servo position to closed
     */
    public void setCapstonePincherClosed () {
        setCapstonePincherPosition(CAPSTONE_PINCHER_CLOSED);
    }

    /**
     * Sets the capstone elbow position
     * @param position - the position of the capstone elbow serov
     */
    public void setCapstoneElbowPosition (double position) {
        capstoneElbow.setPosition(position);
    }

    /**
     * Sets the capstone elbow position to down
     */
    public void setCapstoneElbowDown () {
        setCapstoneElbowPosition(CAPSTONE_ELBOW_DOWN);
    }

    /**
     * Sets the capstone elbow position to up
     */
    public void setCaptstoneElbowUp () {
        setCapstoneElbowPosition(CAPSTONE_ELBOW_UP);
    }

    //**********************************************************************************************
    //
    // END SERVO METHODS
    //
    //**********************************************************************************************


    //**********************************************************************************************
    //
    // COLOR SENSOR METHODS
    //
    //**********************************************************************************************

    /**
     * Gets hue from Color Sensor
     * @return hue
     */
    public float getHue(){

        float hsv[] = {0F, 0F, 0F};
        convertToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
        return hsv[0];
    }

    /**
     * Converts RGB to HSV
     * @param red component
     * @param green component
     * @param blue component
     * @param hsv returned
     */
    public void convertToHSV(int red, int green, int blue, float hsv[]){
        Color.RGBToHSV((int) (red * COLOR_SENSOR_SCALE_FACTOR),
                (int) (green * COLOR_SENSOR_SCALE_FACTOR),
                (int) (blue * COLOR_SENSOR_SCALE_FACTOR),
                hsv);

    }

    //**********************************************************************************************
    //
    // END COLOR SENSOR METHODS
    //
    //**********************************************************************************************


    //**********************************************************************************************
    //
    // LIFT TOUCH SENSOR METHODS
    //
    //**********************************************************************************************

    /**
     * Returns if the lift touch sensor is press (true) or if it is not press (false)
     * @return boolean true - is pressed, false - is not pressed
     */
    public boolean isLiftTouchSensorPressed(){ return liftTouchSensor.isPressed(); }

    //**********************************************************************************************
    //
    // END LIFT TOUCH SENSOR METHODS
    //
    //**********************************************************************************************


    //**********************************************************************************************
    //
    // LIFT MOTOR METHODS
    //
    //**********************************************************************************************

    /**
     * Sets the power of the lift motor
     * @param liftPower - the power to be applied to the lift motor
     */

    public void setLiftMotorPower(double liftPower) { liftMotor.setPower(liftPower); }


    /**
     * Calculates the number of encoder counts to travel a given distance for the lift
     * @param distance
     * @return
     */
    public double liftCalculateCounts(double distance) {

        double COUNTS;

        double DIAMETER = 2.5; //diameter of lift spool
        double GEAR_RATIO = (1.0 / 1.0); //gear ratio
        double PULSES = 1120; //encoder counts in one revolution - neverest 40 motor
        double CIRCUMFERENCE = Math.PI * DIAMETER; //gives you circumference
        double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; //gives the rotations
        COUNTS = PULSES * ROTATIONS; //gives the counts

        return COUNTS;
    }

    /**
     * Sets the encoder target position for the lift motor
     * @param encoderPosition - the encoder position
     */
    public void setLiftMotorTargetPosition(int encoderPosition) {liftMotor.setTargetPosition(encoderPosition);}

    /**
     * Sets the lift motor to the STOP_AND_RESET_ENCODER mode, which will zero the encoder count and
     * set the mode to RUN_WITHOUT_ENCODER.
     */
    public void resetLiftEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Sets the lift motor to the RUN_USING_ENCODER mode
     */
    public void runLiftWithEncoder() {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the lift motor to the RUN_TO_POSITION mode
     */
    public void runLiftWithEncoderRTP() {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Gets the lift motor's encoder current position
     * @return the current position of the lift motor
     */
    public int getCurrentLiftPosition(){return liftMotor.getCurrentPosition();}

    //**********************************************************************************************
    //
    // END LIFT MOTOR METHODS
    //
    //**********************************************************************************************


    //**********************************************************************************************
    //
    // DISTANCE SENSOR METHODS
    //
    //**********************************************************************************************

    /**
     * This will return the distance in centimeters
     * @return distance (CM)
     */
    public double getStoneDistance() {return stoneDistanceSensor.getDistance(DistanceUnit.CM); }

    // public double getRange(){
    //     return stoneRange.getDistance(DistanceUnit.INCH);
    //}

    //**********************************************************************************************
    //
    // END DISTANCE SENSOR METHODS
    //
    //**********************************************************************************************



}


