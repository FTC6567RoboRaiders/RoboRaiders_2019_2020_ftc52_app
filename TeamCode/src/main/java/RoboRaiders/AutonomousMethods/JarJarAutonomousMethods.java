package RoboRaiders.AutonomousMethods;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import RoboRaiders.Robot.JarJarBot;

public abstract class JarJarAutonomousMethods extends LinearOpMode {

    public double motor_power;
    public double degreesToTurn;
    public double currentHeading;
    public double finalHeading;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;


    public void encodersMove(JarJarBot robot, double distance, double power, String direction) { //sets the parameters

        robot.resetEncoders(); //resets encoders
        robot.runWithEncoders(); //sets the mode back to run with encoder

        final double v = robot.calculateCOUNTS(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("forward")) { //if the desired direction is forward

            robot.setDriveMotorPower(power, power, power, power); //start driving forward

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                //telemetry.addData("COUNTS", COUNTS);
                //telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                //telemetry.update();
            }

            robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        } else if (direction.equals("backward")) { //if the desired direction is backward

            robot.setDriveMotorPower(-power, -power, -power, -power); //start driving backward

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }
        }
    }

    public void encodersMoveStrafe(JarJarBot robot, double distance, double power, String direction) {
        robot.resetEncoders();
        robot.runWithEncoders();

        final double v = robot.calculateCOUNTS(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("left")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, -power, power); //start strafing right

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        } else if (direction.equals("right")) { //if the desired direction is left

            robot.setDriveMotorPower(-power, power, power, -power); //start strafing left

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        }

        robot.runWithoutEncoders(); //sets the mode back to run without encoder
    }

    public void runIntake(JarJarBot robot, double power) {
        double startIntakeTime = System.currentTimeMillis();
        robot.intakeMotorLeft.setPower(power);
        robot.intakeMotorRight.setPower(power);
        while (opModeIsActive() && System.currentTimeMillis() - startIntakeTime < 1500) {
        }

        robot.intakeMotorRight.setPower(0.0);
        robot.intakeMotorLeft.setPower(0.0);

    }

    public void robotSleep(int timeToSleep) {
        try {
            Thread.sleep(timeToSleep);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void imuTurnPID(RoboRaidersPID rrPID, JarJarBot robot, float degreesToTurn, String direction) { //gets hardware from
        double power = 0.0;
        int loopcount = 0;


        robot.motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        rrPID.initialize();
        telemetry.addLine().addData("in", "imuTurnPID");

        currentHeading = 0.0;
        currentHeading = robot.getIntegratedZAxis();


        if (direction.equals("right")) { //if the desired direction is right
            finalHeading = currentHeading - degreesToTurn;
            telemetry.addLine().addData("currentHeading", currentHeading);
            telemetry.addLine().addData("finalHeading", finalHeading); // power = rrPID.CalculatePIDPowers(finalHeading,currentHeading);
            telemetry.addLine().addData("power", power);

            while ((opModeIsActive() && (loopcount < 20 &&
                    !(currentHeading < finalHeading + 3.5 && currentHeading > finalHeading - 3.5)))) {
                //&& Math.abs(power) > 0.1) {
                currentHeading = robot.getIntegratedZAxis();
                power = rrPID.CalculatePIDPowers(finalHeading, currentHeading) * 0.75;

                loopcount++;

                robot.setDriveMotorPower(power, power, power, power);

                telemetry.addLine().addData("right", "right");
                telemetry.addLine().addData("power", String.valueOf(power));
                telemetry.addLine().addData("IntZ", String.valueOf(robot.getIntegratedZAxis()));
                telemetry.addLine().addData("finalHeading", String.valueOf(finalHeading));
                telemetry.addLine().addData("difference", String.valueOf(finalHeading - robot.getIntegratedZAxis()));
                telemetry.update();

            }
            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
        } else { //if the desired direction is left
            finalHeading = currentHeading + degreesToTurn;

            L.Debug("Turning Left");
            L.Debug("finalHeading: ", finalHeading);


            while ((opModeIsActive() && (loopcount < 20 &&
                    !(currentHeading > finalHeading - 3.5 && currentHeading < finalHeading + 3.5)))) {
                //&& Math.abs(power) > 0.1) {
                currentHeading = robot.getIntegratedZAxis();
                power = rrPID.CalculatePIDPowers(finalHeading, currentHeading) * 0.75;

                loopcount++;

                L.Debug("In While Loop");
                L.Debug("finalHeading: ", finalHeading);
                L.Debug("currentHeading: ", currentHeading);
                L.Debug("Remaining Degrees: ", finalHeading - currentHeading);
                L.Debug("Calculated PID Power (power): ", power);
                L.Debug("loopcount", loopcount);


                robot.setDriveMotorPower(power, power, power, power);


                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("left", "left");
                //telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                telemetry.addLine().addData("IntZ", String.valueOf(robot.integratedZAxis));
                telemetry.addLine().addData("finalHeading", String.valueOf(finalHeading));
                telemetry.addLine().addData("difference", String.valueOf(finalHeading - robot.getIntegratedZAxis()));
                telemetry.update();
            }
        }


        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot

        robot.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);


        L.Debug("End");
    }
}
