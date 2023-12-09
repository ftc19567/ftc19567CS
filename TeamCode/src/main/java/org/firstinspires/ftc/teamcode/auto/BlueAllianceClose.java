package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.PropHSVPipelineBlue;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="BlueAllianceCloseAutonomous")
public class BlueAllianceClose extends LinearOpMode {
    private Servo turnServo;
    private Arm arm;
    private Intake intake;

    OpenCvCamera camera;
    WebcamName webcam1;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        turnServo = hardwareMap.get(Servo.class, "turnServo");
        arm = new Arm(hardwareMap, telemetry);


        turnServo.setPosition(0.8);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(webcam1, cameraMonitorViewId);

        PropHSVPipelineBlue pipeline = new PropHSVPipelineBlue(telemetry);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });

        //left

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(new Pose2d(9.5, 61.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d (22, 40, Math.toRadians(90)))
                .back(-5)
                .splineToLinearHeading(new Pose2d(40, 42, Math.toRadians(-180)), Math.toRadians(0))
                .build();


        //middle

        TrajectorySequence middleTraj = drive.trajectorySequenceBuilder(new Pose2d(9.5, 61.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d (12, 33, Math.toRadians(90)))
                .back(-5)
                .splineToLinearHeading(new Pose2d(40, 36, Math.toRadians(-180)), Math.toRadians(0))
                .build();

        //right
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(new Pose2d(9.5, 61.5, Math.toRadians(90)))
                .lineTo(new Vector2d(14, 34))
                .turn(Math.toRadians(-70))
                .forward(-5)
                .splineToLinearHeading(new Pose2d(40, 28, Math.toRadians(180)), Math.toRadians(0))
                .build();

        //park

        TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(leftTraj.end())
                .waitSeconds(0.5)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                .forward(10)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(1))
                .splineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence middleTraj1 = drive.trajectorySequenceBuilder(middleTraj.end())
                .waitSeconds(0.5)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                .forward(10)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(1))
                .splineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(rightTraj.end())
                .waitSeconds(0.5)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                .forward(10)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(1))
                .splineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)), Math.toRadians(0))
                .build();










        waitForStart();

        switch(PropHSVPipelineBlue.getLocation()) {

            case LEFT:
                drive.setPoseEstimate(new Pose2d(9.5, 61.5, Math.toRadians(90)));

                drive.followTrajectorySequence(leftTraj);


                turnServo.setPosition(1);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm.setPosition(0.6, 1856);
                try {
                    Thread.sleep(700);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnServo.setPosition(0.11000000000000004);

                drive.followTrajectorySequence(leftTraj1);



                break;



            case MIDDLE:
                drive.setPoseEstimate(new Pose2d(9.5, 61.5, Math.toRadians(90)));

                drive.followTrajectorySequence(middleTraj);


                turnServo.setPosition(1);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm.setPosition(0.6, 1856);
                try {
                    Thread.sleep(700);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnServo.setPosition(0.11000000000000004);

                drive.followTrajectorySequence(middleTraj1);

                break;

            case RIGHT:
                drive.setPoseEstimate(new Pose2d(9.5, 61.5, Math.toRadians(90)));

                drive.followTrajectorySequence(rightTraj);


                turnServo.setPosition(1);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm.setPosition(0.6, 1856);
                try {
                    Thread.sleep(700);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnServo.setPosition(0.11000000000000004);

                drive.followTrajectorySequence(rightTraj1);



                break;

            case NONE_DETECTED:
                break;
        }
        camera.stopStreaming();
        telemetry.addLine("COMPLETE");

    }

}
