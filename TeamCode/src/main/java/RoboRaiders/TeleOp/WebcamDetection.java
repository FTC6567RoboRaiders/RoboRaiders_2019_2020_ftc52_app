package RoboRaiders.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import RoboRaiders.Autonomous.RoboRaidersPipelineWebcam;

import static org.opencv.core.CvType.CV_8UC1;

@TeleOp
public class WebcamDetection extends LinearOpMode {
    public static final String TAG = "Vuforia Navigation Sample";

    OpenCvCamera webcam;
   // BrightnessDetection.SamplePipeline stone_pipeline;
    RoboRaidersPipelineWebcam stone_pipeline;
    public void runOpMode() {
        float leftRec[] = {9f, 5f,15f,15f};
        float rightRec[] = {9f, 17f, 15f, 27f};

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();
        stone_pipeline = new RoboRaidersPipelineWebcam(999,leftRec,rightRec);
        webcam.setPipeline(stone_pipeline);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        waitForStart();

        while (opModeIsActive()) {
            super.updateTelemetry(telemetry);
            telemetry.addData("PATTERN", stone_pipeline.pattern);
        }

        webcam.stopStreaming();

     }
}