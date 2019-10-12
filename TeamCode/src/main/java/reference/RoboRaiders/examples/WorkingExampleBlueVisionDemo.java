package RoboRaiders.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

import RoboRaiders.Logger.Logger;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 */
@TeleOp(name="Example: Working Blue Vision Demo")
@Disabled
public class WorkingExampleBlueVisionDemo extends OpMode {
    private WorkingExampleBlueVision blueVision;
    private Logger l = new Logger("JQ");
    @Override
    public void init() {
        blueVision = new WorkingExampleBlueVision();
        telemetry.setAutoClear(false);
        telemetry.addLine("created WorkingExampleBlueVision");
        telemetry.update();

        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        telemetry.addLine("blueVision inited");
        telemetry.update();

        telemetry.addLine("blueVision showcontours set");
        telemetry.update();

        // start the vision system
        blueVision.enable();
        telemetry.addLine("blueVision enabled");
        telemetry.update();

    }

    @Override
    public void loop() {




        // update the settings of the vision pipeline
        blueVision.setShowCountours(gamepad1.x);

        // get a list of contours from the vision system
        Mat image = blueVision.getImage();

        telemetry.setAutoClear(true);
        telemetry.update();
        telemetry.setAutoClear(false);

        telemetry.addLine("outputting data");

        //    for (int i = 0; i < contours.size(); i++) {
        // get the bounding rectangle of a single contour, we use it to get the x/y center
        // yes there's a mass center using Imgproc.moments but w/e
    //    Rect boundingRect = Imgproc.boundingRect(contour);

      //  StringBuilder outdata = new StringBuilder();

        for (int i=0; i < image.height(); i++) {
            for (int j=0; j<image.width(); j++) {
                double[] data = image.get(i,j);
                l.Debug(String.valueOf("data("+i+","+j+"): "),data[0],data[1],data[2]);
            }
         //   outdata.append(data[i]);
         //   outdata.append(",");
        }

        telemetry.addLine("done outputting data");

       /* telemetry.addLine(outdata.toString());
        telemetry.addLine(String.valueOf(data.length));
        telemetry.addLine(String.valueOf(image.height()+","+image.width())); */

      /*  telemetry.addData(
                "contour",
                String.format(
                        Locale.getDefault(),
                        "(%d, %d)",
                        (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2
                )
        ); */

        //  }


    }

    public void stop() {
        // stop the vision system
        blueVision.disable();
    }
}
