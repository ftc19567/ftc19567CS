package org.firstinspires.ftc.teamcode.auto.vision.OpenCV;

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
public class OpenCVFirst extends OpMode {

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
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }

    @Override
    public void loop() {

    }

    public class examplePipeline extends OpenCvPipeline {
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

            Rect leftRect = new Rect(1, 1, 426, 719);
            Rect middleRect = new Rect(427, 1, 425, 719);
            Rect rightRect = new Rect(852, 1, 426, 719);

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

            telemetry.addLine("leftavgfin" + leftavgfin);
            telemetry.addLine("middleavgfin" + middleavgfin);
            telemetry.addLine("rightavgfin" + rightavgfin);

            if (leftavgfin > middleavgfin && leftavgfin > rightavgfin) {
                telemetry.addLine("left");
            } else if (rightavgfin > leftavgfin && rightavgfin > middleavgfin) {
                telemetry.addLine("right");
            } else {
                telemetry.addLine("middle");


            }
            telemetry.update();
            return (outPut);


        }
    }
}