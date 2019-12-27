package RoboRaiders.Robot;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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

    public BNO055IMU imu;

 //   public DistanceSensor stoneRange = null;

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
    private static final double FOUNDATION_SERVO_GRAB_LEFT = 0.5;
    private static final double FOUNDATION_SERVO_GRAB_RIGHT = 0.9;
    private static final double FOUNDATION_SERVO_RELEASE_LEFT = 1.0;
    private static final double FOUNDATION_SERVO_RELEASE_RIGHT = 0.35;

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

        //stoneRange = hwMap.get(DistanceSensor.class, "sensor_range");

        // Defines the directions the motors will spin
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorRight.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);

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
    }

        public ModernRoboticsI2cRangeSensor mrDistance;
  //      Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)stoneRange;

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

    public void setLiftMotorPower(double liftPower) { liftMotor.setPower(liftPower); }


    //public void takeSkystoneDown(){ takeSkystone.setPosition(1.0);}
    //public void takeSkystoneUp() {takeSkystone.setPosition(0.0);}

    /**
     * Calculates the number of encoder counts to travel a given distance
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
    public double getSensorDistance() {

        return mrDistance.getDistance(DistanceUnit.INCH);

    }

    public void setDTMotorTargetPosition(int encoderPosition){
        motorFrontLeft.setTargetPosition(encoderPosition);
        motorFrontRight.setTargetPosition(encoderPosition);
        motorBackLeft.setTargetPosition(encoderPosition);
        motorBackRight.setTargetPosition(encoderPosition);
    }
    /**
     * This method will set the mode of all of the motors to run using encoder
     */
    public void runWithEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

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
    public void resetEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public float getHeading() {

        float heading;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
        heading = Math.abs(angles.firstAngle); //heading is equal to the absolute value of the first angle

        return heading;
    }

   // public double getRange(){
   //     return stoneRange.getDistance(DistanceUnit.INCH);
   //}

    public void resetIMU() {

        imu.initialize(parameters);
    }

    public double getBackLeftDriveEncoderCounts() { return motorBackLeft.getCurrentPosition(); }
    public double getBackRightDriveEncoderCounts() { return motorBackRight.getCurrentPosition(); }
    public double getFrontLeftDriveEncoderCounts() { return motorFrontLeft.getCurrentPosition(); }
    public double getFrontRightDriveEncoderCounts() { return motorFrontRight.getCurrentPosition(); }


    /**
     * Initialize the Vuforia localization engine.
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
    public void setInakePower(double intake){
        intakeMotorLeft.setPower(intake);
        intakeMotorRight.setPower(-1*intake);
    }

    public void setCaptureServoPosition (double position) {

        stoneCaptureServo.setPosition(position);
    }

    public void setCaptureServoUp () {

        setCaptureServoPosition(CAPTURE_SERVO_UP); //Captures the stone
    }

    public void setCaptureServoDown (){

        setCaptureServoPosition(CAPTURE_SERVO_DOWN);
    }

    public void setStoneSwingPosition (double position) {

        stoneSwingServo.setPosition(position);
    }

    public void setStoneSwingServoOut () {

        setStoneSwingPosition(SWING_SERVO_OUT);
    }

    public void setStoneSwingServoIn () {

        setStoneSwingPosition(SWING_SERVO_IN);
    }

    public void setfoundationGrabberPostion (double posLeft, double posRight) {
        foundationGrabberLeft.setPosition(posLeft);
        foundationGrabberRight.setPosition(posRight);
    }

    public void setFoundationGrabberGrabbed () {
        setfoundationGrabberPostion(FOUNDATION_SERVO_GRAB_LEFT, FOUNDATION_SERVO_GRAB_RIGHT);

    }
    public void setFoundationGrabberUnGrabbed () {
        setfoundationGrabberPostion(FOUNDATION_SERVO_RELEASE_LEFT, FOUNDATION_SERVO_RELEASE_RIGHT);
    }


 }


