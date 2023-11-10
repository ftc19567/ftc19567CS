package org.firstinspires.ftc.teamcode.auto.vision.OpenCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetectorPipeline extends OpenCvPipeline {
    Telemetry telemetry; // Info
    Mat mat = new Mat(); // Matrix of the Camera (essentially the camera image)

    // Creating ROIs for the camera (Regions of Interest) to see where the object is located.
    static final Rect LEFT_ROI = new Rect (
            new Point(1, 1),
            new Point(300, 300));

    static final Rect RIGHT_ROI = new Rect (
            new Point(301, 1),
            new Point(600, 300));


    public TeamPropDetectorPipeline (Telemetry t) { telemetry = t;} //Passing telemetry as a constructor

    @Override
    public Mat processFrame(Mat input) {
        //Converting the Matrix from rgb to HSV (Hue, Saturation, and Value/Brightness)
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //lower hsv values to detect the color. Values can be modified and tuned as necessary.
        Scalar lowHSVBlue = new Scalar (221, 68.05, 42);
        // same here. Values haven't been tuned yet.
        Scalar highHSVBlue = new Scalar (235, 100, 100);

        //Thresholding, turning pixels meeting the hsv range into white and the rest into black.
        Core.inRange(mat, lowHSVBlue, highHSVBlue, mat);

        //Extracting the ROIs from the image
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        //Finding the percent of the white pixels. Determine it by adding the white pixels together (.val [0} finds them) and
        // dividing by the total area and the max greyscale, 255.
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        //Prevents memory leaking by release the Mat right after using them.
        left.release();
        right.release();

        //telemtry for info
        telemetry.addData("Left raw value: ", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value: ", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage: ", Math.round(leftValue *100) + "%");
        telemetry.addData("Right percentage: ", Math.round(rightValue *100) + "%");

        telemetry.update();


        return mat;

    }
}
