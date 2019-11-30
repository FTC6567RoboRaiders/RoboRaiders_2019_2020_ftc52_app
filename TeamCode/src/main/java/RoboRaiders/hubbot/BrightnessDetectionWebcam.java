package RoboRaiders.hubbot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import easyopencv.examples.WebcamExample;

import static org.opencv.core.CvType.CV_8UC1;

@TeleOp
public class BrightnessDetectionWebcam extends LinearOpMode {
    public static final String TAG = "Vuforia Navigation Sample";

    OpenCvCamera webcam;
   // BrightnessDetection.SamplePipeline stone_pipeline;
    SamplePipeline stone_pipeline;
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();
        stone_pipeline = new SamplePipeline();
        webcam.setPipeline(stone_pipeline);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        waitForStart();

        while (opModeIsActive()) {
        super.updateTelemetry(telemetry);
        telemetry.addData("LEFT RECT", stone_pipeline.left_hue + " " + stone_pipeline.left_br);
        telemetry.addData("RIGHT RECT", stone_pipeline.right_hue + " " + stone_pipeline.right_br);
        telemetry.addData("PATTERN", stone_pipeline.pattern); }


     }

    class SamplePipeline extends OpenCvPipeline {
        int left_hue;
        int right_hue;

        int left_br;
        int right_br;

        int pattern;

        @Override
        public Mat processFrame(Mat input) {

            input.convertTo(input, CV_8UC1, 1, 10);

            int[] left_rect = {
                    //the second number here possibly represents the amount of sections you are splitting the screen into??   32
                    (int) (input.cols() * (9f / 32f)), //the first number goes from left to right increasing, controls x axis
                    (int) (input.rows() * (5f / 32f)), //the first number here controls the y axis
                    (int) (input.cols() * (15f / 32f)),
                    (int) (input.rows() * (15f / 32f))
            };
//because we rotated the camera, collumns are now rows and vice versa. Rows are now on the x-axis!
            int[] right_rect = {
                    (int) (input.cols() * (9f / 32f)),
                    (int) (input.rows() * (17f / 32f)),
                    (int) (input.cols() * (15f / 32f)),
                    (int) (input.rows() * (27f / 32f))
            };

            Imgproc.rectangle(
                    input,
                    new Point(
                            left_rect[0],
                            left_rect[1]),

                    new Point(
                            left_rect[2],
                            left_rect[3]),
                    new Scalar(0, 255, 0), 1);

            Imgproc.rectangle(
                    input,
                    new Point(
                            right_rect[0],
                            right_rect[1]),

                    new Point(
                            right_rect[2],
                            right_rect[3]),
                    new Scalar(0, 0, 255), 1);

            Mat left_block = input.submat(left_rect[1], left_rect[3], left_rect[0], left_rect[2]);
            Mat right_block = input.submat(right_rect[1], right_rect[3], right_rect[0], right_rect[2]);


            Scalar left_mean = Core.mean(left_block);


            Scalar right_mean = Core.mean(right_block);

            left_hue = get_hue((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
            right_hue = get_hue((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);
            left_br = get_brightness((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
            right_br = get_brightness((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);

            if (left_br > 100 && right_br > 100) pattern = 1;
            //skystone is not in frame
            //above 100 is normal, bellow 100 is skystone
            else if (left_br > 100 && right_br < 100) pattern = 2;
            //skystone is on left
            else if (left_br < 100 && right_br > 100) pattern = 3;
            //skystone is on right
            else if (left_br < 100 && right_br < 100) {
                if (left_br > right_br) {
                    pattern = 1;
                } else if (left_br < right_br) {
                    pattern = 2;
                } else {
                    pattern = 3;
                }
            }
                telemetry.addData("position", pattern);
                telemetry.update();
                sleep(100);

            return input;
        }
    }


    private int get_hue(int red, int green, int blue) {

        float min = Math.min(Math.min(red, green), blue);
        float max = Math.max(Math.max(red, green), blue);

        if (min == max) {
            return 0;
        }

        float hue = 0f;
        if (max == red) {
            hue = (green - blue) / (max - min);

        } else if (max == green) {
            hue = 2f + (blue - red) / (max - min);

        } else {
            hue = 4f + (red - green) / (max - min);
        }

        hue = hue * 60;
        if (hue < 0) hue = hue + 360;

        return Math.round(hue);
    }

    private int get_brightness(int red, int green, int blue) {
        return (int) (((double) (red + green + blue)) / 3);
    }
}