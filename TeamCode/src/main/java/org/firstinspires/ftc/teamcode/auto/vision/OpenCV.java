package org.firstinspires.ftc.teamcode.auto.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name="OpenCVTest")
public class OpenCV extends OpMode {

    OpenCvWebcam camera = null;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(new examplePipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }
    @Override
    public void loop() {

    }

    public class examplePipeline extends OpenCvPipeline{
        Mat YCBCR = new Mat();
        Mat leftCrop;
        Mat middleCrop;
        Mat rightCrop;
        double leftavgfin;
        double middleavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCBCR, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 1, 213, 359);
            Rect middleRect = new Rect(214, 1, 214, 359);
            Rect rightRect = new Rect(429, 1, 213, 359);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, middleRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = YCBCR.submat(leftRect);
            middleCrop = YCBCR.submat(middleRect);
            rightCrop = YCBCR.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar middleavg = Core.mean(middleCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            middleavgfin = middleavg.val[0];
            rightavgfin = rightavg.val[0];

            if (leftavgfin > middleavgfin && leftavgfin > rightavgfin) {
                telemetry.addLine("left");
            } else if (middleavgfin > leftavgfin && middleavgfin > rightavgfin) {
                telemetry.addLine("middle");
            } else {
                telemetry.addLine("right");
            }



            return(outPut);

        }
    }
}