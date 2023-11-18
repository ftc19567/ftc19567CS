package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.PropHSVPipelineBlue;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="BlueCloseTest")
public class BlueCloseTestAuto extends LinearOpMode {
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




            Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                    .forward(-58)
                    .build();

            Trajectory backward = drive.trajectoryBuilder(forward.end())
                    .forward(49)
                    .build();

            Trajectory rightforward = drive.trajectoryBuilder(backward.end())
                    .forward(-40)
                    .build();

            Trajectory rightSpike = drive.trajectoryBuilder(backward.end())
                    .forward(-29)
                    .build();

            Trajectory board = drive.trajectoryBuilder(new Pose2d())
                    .forward(-73)
                    .build();

            Trajectory middleboarddrive = drive.trajectoryBuilder(new Pose2d())
                    .forward(-80)
                    .build();

            Trajectory boardstrafesmall = drive.trajectoryBuilder(board.end())
                    .strafeLeft(47)
                    .build();

            Trajectory middleforward = drive.trajectoryBuilder(new Pose2d())
                    .forward(-65)
                    .build();


            Trajectory mediumoardstrafe = drive.trajectoryBuilder(board.end())
                    .strafeLeft(20)
                    .build();

            Trajectory dropPixel = drive.trajectoryBuilder(board.end())
                    .forward(-20)
                    .build();

            Trajectory rightboardstrafe = drive.trajectoryBuilder(board.end())
                    .strafeLeft(30)
                    .build();






            waitForStart();

            switch(PropHSVPipelineBlue.getLocation()) {

                case LEFT:
                    drive.turn(Math.toRadians(70));
                    drive.followTrajectory(forward);
                    drive.followTrajectory(backward);
                    drive.turn(Math.toRadians(140));
                    drive.followTrajectory(board);
                    drive.followTrajectory(boardstrafesmall);
                    drive.turn(Math.toRadians(-40));

                    //arm
                    turnServo.setPosition(1);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(0.7, 1826);
                    try {
                        Thread.sleep(700);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(0.25);

                    drive.followTrajectory(dropPixel);

                    turnServo.setPosition(1);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(1, -16);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(0.8);


                    break;


                case MIDDLE:
                    drive.turn(Math.toRadians(-40));
                    drive.followTrajectory(middleforward);
                    drive.followTrajectory(backward);
                    drive.turn(Math.toRadians(140));
                    drive.followTrajectory(middleboarddrive);
                    drive.followTrajectory(mediumoardstrafe);
                    drive.turn(Math.toRadians(60));

                    turnServo.setPosition(1);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(0.7, 1826);
                    try {
                        Thread.sleep(700);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(0.25);

                    drive.followTrajectory(dropPixel);

                    turnServo.setPosition(1);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(1, -16);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(0.8);
                    //drive.followTrajectory(middle2);
                    break;

                case RIGHT:
                    drive.followTrajectory(rightforward);
                    drive.turn(Math.toRadians(-125));
                    drive.followTrajectory(rightSpike);
                    drive.followTrajectory(backward);
                    drive.turn(Math.toRadians(200));
                    drive.followTrajectory(board);
                    drive.followTrajectory(rightboardstrafe);
                    drive.turn(Math.toRadians(50));

                    turnServo.setPosition(1);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(0.7, 1826);
                    try {
                        Thread.sleep(700);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(0.25);

                    drive.followTrajectory(dropPixel);

                    turnServo.setPosition(1);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(1, -16);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(0.8);
                case NONE_DETECTED:
                    break;
            }
            camera.stopStreaming();
            telemetry.addLine("COMPLETE");

        }

    }
