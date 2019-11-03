//package RoboRaiders.Tests;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//
//@TeleOp(name="TurningTest", group="Samples")
//@Disabled
//
//public class TurningTest extends LinearOpMode {
//    public float degreesToTurn;
//    public float currentHeading;
//    public float finalHeading;
//    public double integratedZAxis;
//    public double iza_lastHeading = 0.0;
//    public double iza_deltaHeading;
//    public float iza_newHeading;
//    public Orientation iza_angles;
//    Orientation angles;
//    BNO055IMU imu;
//
//
//
//
//    @Override
//    public void runOpMode() {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//
//        telemetry.update();
//
//        waitForStart();
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        degreesToTurn = 190;
//        //telemetry.addLine().addData("degreesToTurn",String.valueOf(degreesToTurn));
//        currentHeading = angles.firstAngle;
//        finalHeading = currentHeading + degreesToTurn;
//        //telemetry.update();
//
//
//        while(opModeIsActive() && getIntegratedZAxis() < finalHeading) {
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            currentHeading = angles.firstAngle;
//            telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
//            telemetry.addLine().addData("IntZ",String.valueOf(integratedZAxis));
//            telemetry.update();
//        }
//
//    }
//
//    public double getIntegratedZAxis() {
//
//        // This sets up the how we want the IMU to report data
//        iza_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        // Obtain the heading (Z-Axis)
//        iza_newHeading = iza_angles.firstAngle;
//
//        // Calculate the change in the heading from the previous heading
//        iza_deltaHeading = iza_newHeading - iza_lastHeading;
//
//        // Bosch IMU wraps at 180, so compensate for this
//        if (iza_deltaHeading <= -180.0) {
//
//            iza_deltaHeading += 360.0;
//        }
//        else if (iza_deltaHeading >= 180.0) {
//
//            iza_deltaHeading -= 360;
//        }
//
//        // Calculate the integratedZAxis
//        integratedZAxis += iza_deltaHeading;
//
//        // Save the current heading for the next call to this method
//        iza_lastHeading = iza_newHeading;
//
//        return integratedZAxis;
//    }
//}
//
