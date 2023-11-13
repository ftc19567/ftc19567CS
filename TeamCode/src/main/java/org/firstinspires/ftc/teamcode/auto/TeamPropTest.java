package org.firstinspires.ftc.teamcode.auto;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


import org.firstinspires.ftc.teamcode.auto.vision.OpenCV.TeamPropHSVPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="AutoTestTele")
public class TeamPropTest extends LinearOpMode {
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

            TeamPropHSVPipeline pipeline = new TeamPropHSVPipeline(telemetry);

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

            Trajectory middle2 = drive.trajectoryBuilder(backward.end())
                    .splineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(0)), Math.toRadians(90))
                    .build();

            Trajectory board = drive.trajectoryBuilder(new Pose2d())
                    .forward(-90)
                    .build();

            Trajectory boardstrafesmall = drive.trajectoryBuilder(board.end())
                    .strafeLeft(65)
                    .build();

            Trajectory boardstrafemedium = drive.trajectoryBuilder(board.end())
                    .strafeLeft(72)
                    .build();

            Trajectory boardstrafelarge = drive.trajectoryBuilder(board.end())
                    .strafeLeft(80)
                    .build();

            Trajectory dropPixel = drive.trajectoryBuilder(board.end())
                    .forward(-30)
                    .build();






            waitForStart();

            switch(TeamPropHSVPipeline.getLocation()) {

                case LEFT:
                    drive.turn(Math.toRadians(70));
                    drive.followTrajectory(forward);
                    drive.followTrajectory(backward);
                    drive.turn(Math.toRadians(130));
                    drive.followTrajectory(board);
                    drive.followTrajectory(boardstrafesmall);

                    //arm
                    turnServo.setPosition(1);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(0.8, 1600);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(0.36);

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
                    drive.followTrajectory(forward);
                    drive.followTrajectory(backward);
                    drive.turn(Math.toRadians(180));
                    drive.followTrajectory(board);
                    drive.followTrajectory(boardstrafemedium);

                    turnServo.setPosition(1);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(0.8, 1600);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(0.36);

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
                    drive.turn(Math.toRadians(-80));
                    drive.followTrajectory(forward);
                    drive.followTrajectory(backward);
                    drive.turn(Math.toRadians(180));
                    drive.followTrajectory(board);
                    drive.followTrajectory(boardstrafelarge);

                    turnServo.setPosition(1);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(0.8, 1600);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(0.36);

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
