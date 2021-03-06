package RoboRaiders.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

/**
 * Created by Steve Kocik.
 */

public class NostromoBot {

    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorLift = null;
    public DcMotor liftIntake = null;
    public BNO055IMU imu;
    public TouchSensor sensorTouch;
    public Servo liftClaw = null;
    public Servo markerDrop = null;
    public Servo intake = null;
    public Servo dumpp1 = null;
    public Servo dumpp2 = null;
    public Servo slider = null;
    public Servo intakeDoor = null;
    public Servo dumpWrist = null;


//me too and because im stupid i didnt do any of it before work
    //I shouldn't (be)complain(ing) tho im sure I don't have as much as you
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
    public double liftClawOpen = 1.0;
    public double liftClawClosed = 0.0;
    public double markerDropUp = 0.3;
    public double markerDropDown = 1.0;
    public double intakeIn = 1.0;
    public double intakeOut = 0.0;
    public double dumpdirection1 = 1.0;
    public double dumpdirection2 = 0.0;
    public double dumpdirectionstop = 0.5;
    public double sliderdirectionout = 1.0;
    public double sliderdirectionin = 0.0;
    public double liftIntakedirectionup = 1.0;
    public double liftIntakedirectiondown = 0.0;
    public double intakeDoorOpen = 0.0;
    public double intakeDoorClosed = 1.0;
    public double dumpWristDump = .1;
    public double dumpWristNotDump = 0.0;
    public double dropTeamMarker = 0.5;
    public double bringMarkerBack = 0.1;
    public double dumpTeamMarkerWristDump = 1.0;
    boolean robotDown;

    private static final String VUFORIA_KEY = "AedUDNP/////AAAAGXH2ZpUID0KanSX9ZSR37LKFSFokxIqmy/g0BNepdA9EepixxnO00qygLnMJq3Fg9gZxnkUJaKgk14/UjhxPWVQIs90ZXJLc21NvQvOeZ3dOogagVP8yFnFQs2xCijGmC/CE30ojlAnbhAhqz1y4tZPW2QkK5Qt0xCakTTSAw3KPQX2mZxX+qMxI2ljrN0eaxaKVnKnAUl8x3naF1mez7f9c8Xdi1O5auL0ePdG6bJhWjEO1YwpSd8WkSzNDEkmw20zpQ7zaOOPw5MeUQUr9vAS0fef0GnLjlS1gb67ajUDlEcbbbIeSrLW/oyRGTil8ueQC2SWafdspSWL3SJNaQKWydies23BxJxM/FoLuYYjx";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";



    /**
     * Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     */
    public NostromoBot() {

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
        motorLift = hwMap.get(DcMotor.class, "motorLift");
        liftIntake = hwMap.get(DcMotor.class,"liftintake");
        sensorTouch = hwMap.touchSensor.get("sensorTouch");
        liftClaw = hwMap.servo.get("liftClaw");
        //markerDrop= hwMap.servo.get("markerDrop");
        intake = hwMap.servo.get("intake");
        dumpp1 = hwMap.servo.get("dumpp1");
        dumpp2 = hwMap.servo.get("dumpp2");
        slider = hwMap.servo.get("slider");
        intakeDoor = hwMap.servo.get("intakedoor");
        dumpWrist = hwMap.servo.get("dumpwrist");




        // Defines the directions the motors will spin
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        liftIntake.setDirection(DcMotor.Direction.FORWARD);
        liftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//have the motors on the drivetrain break here.
        // Set all motors to zero power
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);



        liftClaw.setPosition(liftClawClosed);
       // markerDrop.setPosition(markerDropUp);
        intake.setPosition(0.5);
        dumpp1.setPosition(0.5);
        dumpp2.setPosition(0.5);
        slider.setPosition(0.5);
        intake.setPosition(0.5);
        intakeDoor.setPosition((intakeDoorClosed));
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODER if encoders are installed, and we wouldn't use encoders for teleop, even if we
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //will use them in teleop.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        // Define and initialize sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);
    }
        public ModernRoboticsI2cRangeSensor mrDistance;

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
        motorFrontRight.setPower(rightFront);
        motorBackLeft.setPower(leftBack);
        motorBackRight.setPower(rightBack);
    }


    public void setLiftMotorPower(double lift){
        motorLift.setPower(lift);
    }

    public void setLiftIntakePower(double liftintake){
        liftIntake.setPower(liftintake);
    }

    public void collectionIn() {   intake.setPosition(1.0); }

    public void collectionOut() {   intake.setPosition(0.0); }

    public void collectionOff() {   intake.setPosition(0.5); }

    public void closeIntakeDoor () {
        intakeDoor.setPosition(intakeDoorClosed);
    }

    public void openIntakeDoor () {
        intakeDoor.setPosition(intakeDoorOpen);
    }

    public void pushIntakein () {
        slider.setPosition(sliderdirectionin);
    }

    public void pushIntakeout () {
        slider.setPosition(sliderdirectionout);
    }

    public void dumperUp() {

        dumpp1.setPosition(dumpdirection1);
        dumpp2.setPosition(dumpdirection2);
    }

    public void dumperDown() {

        dumpp1.setPosition(dumpdirection2);
        dumpp2.setPosition(dumpdirection1);
    }

    public void dumperStop() {

        dumpp1.setPosition(0.5);
        dumpp2.setPosition(0.5);
    }




    public double calculateCOUNTS(double distance) {

        double COUNTS;

        int DIAMETER = 4; //diameter of wheel
        double GEAR_RATIO = (2.0 / 3.0); //gear ratio
        int PULSES = 1120; //encoder counts in one revolution
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
    /**
     * This method will set the mode of all of the motors to run using encoder
     */
    public void runWithEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
 }


