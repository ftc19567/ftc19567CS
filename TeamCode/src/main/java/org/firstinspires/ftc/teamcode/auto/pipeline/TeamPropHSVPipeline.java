package org.firstinspires.ftc.teamcode.auto.pipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropHSVPipeline extends OpenCvPipeline {
    Telemetry telemetry; // Info
    Mat mat = new Mat(); // Matrix of the Camera (essentially the camera image)


    // Creating ROIs for the camera (Regions of Interest) to see where the object is located.
    static final Rect LEFT_ROI = new Rect (
            new Point(1, 1),
            new Point(200, 359));

    static final Rect MIDDLE_ROI = new Rect (
            new Point(201, 1),
            new Point(438, 359));

    static final Rect RIGHT_ROI = new Rect (
            new Point(439, 1),
            new Point(639, 359));


    //lower hsv values to detect the color. Values can be modified and tuned as necessary.
    Scalar lowHSVBlue = new Scalar (46.8, 96.3, 41.1);
    // same here. Values can be tuned using eocv sim in valueFinder
    Scalar highHSVBlue = new Scalar (137.4, 255, 255);

    //Defining colors for the ROI, thhis is just design.

    Scalar rectColor = new Scalar(255.0, 0.0, 0.0);


    public TeamPropHSVPipeline (Telemetry t) { telemetry = t;} //Passing telemetry as a constructor

    @Override
    public Mat processFrame(Mat input) {
        //Converting the Matrix from rgb to HSV (Hue, Saturation, and Value/Brightness)
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Thresholding, turning pixels meeting the hsv range into white and the rest into black.
        Core.inRange(mat, lowHSVBlue, highHSVBlue, mat);

        //Extracting the ROIs from the image
        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        //Finding the percent of the white pixels. Determine it by adding the white pixels together (.val [0} finds them) and
        // dividing by the total area and the max greyscale, 255.
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        //Prevents memory leaking by release the Mat right after using them.
        left.release();
        middle.release();
        right.release();



        //So you can see the ROIs
        Imgproc.rectangle(mat, LEFT_ROI, rectColor, 2);
        Imgproc.rectangle(mat, MIDDLE_ROI, rectColor, 2);
        Imgproc.rectangle(mat, RIGHT_ROI, rectColor, 2);

        //Rounding the percents

        int leftPercent = (int) Math.round(leftValue *100);
        int middlePercent = (int) Math.round(middleValue *100);
        int rightPercent = (int) Math.round(rightValue *100);


        //telemtry for info
        //telemetry.addData("Left raw value: ", (int) Core.sumElems(left).val[0]);
        //telemetry.addData("Right raw value: ", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage: ", leftPercent + "%");
        telemetry.addData("Middle percentage: ", middlePercent + "%");
        telemetry.addData("Right percentage: ", rightPercent + "%");

        if (leftPercent == middlePercent && leftPercent == rightPercent) {
            telemetry.addData("Side: ", "None Detected");
        } else if (leftPercent > middlePercent && leftPercent > rightPercent) {
            telemetry.addData("Side: ", "Left");
        } else if (rightPercent > leftPercent && rightPercent > middlePercent) {
            telemetry.addData("Side: ", "Right");
        } else {
            telemetry.addData("Side: ", "Middle");


        }

        //so telemetry won't clutter
        telemetry.update();



        //return the mat to the ui
        return mat;

    }
}