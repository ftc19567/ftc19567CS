package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.PropHSVPipelineBlue;
import org.firstinspires.ftc.teamcode.auto.pipeline.PropHSVPipelineRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
//import org.firstinspires.ftc.teamcode.mechanisms.ThreadAD;
//import org.firstinspires.ftc.teamcode.mechanisms.ThreadAU;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="RedAllianceCloseAuto")
public class RedAllianceClose extends LinearOpMode {
        private Servo turnServo;
        private Arm arm;
        private Intake intake;
        private DcMotor intakeMotor;
        OpenCvCamera camera;
        WebcamName webcam1;



        private void armUP() {
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
        }

        private void armDown() {
            turnServo.setPosition(1);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            arm.setPosition(1, 5);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            turnServo.setPosition(0.7);
        }

        @Override
        public void runOpMode() throws InterruptedException {

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            turnServo = hardwareMap.get(Servo.class, "turnServo");
            arm = new Arm(hardwareMap, telemetry);

            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");


            int cameraMonitorViewId = hardwareMap.appContext
                    .getResources().getIdentifier("cameraMonitorViewId",
                            "id", hardwareMap.appContext.getPackageName());
            webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
            camera = OpenCvCameraFactory.getInstance()
                    .createWebcam(webcam1, cameraMonitorViewId);

            PropHSVPipelineRed pipeline = new PropHSVPipelineRed(telemetry);

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

            TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(new Pose2d(14.5, -61.5, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d (7, -34, Math.toRadians(-20)))
                    .back(-5)
                    .splineToLinearHeading(new Pose2d(40, -28, Math.toRadians(180)), Math.toRadians(0))
                    .build();

            TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(leftTraj.end())
                    .waitSeconds(0.25)
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .forward(10)
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(0.7))
                    .splineToLinearHeading(new Pose2d(30, -60, Math.toRadians(180)), Math.toRadians(180))
                    .waitSeconds(0.25)
                    .lineToLinearHeading(new Pose2d(-13, -60, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-12.5, -10, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    .lineToLinearHeading(new Pose2d(-58, -11, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> intakeMotor.setPower(1))
                    .forward(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeMotor.setPower(0))
                    .lineToLinearHeading(new Pose2d(45, -10, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    //.UNSTABLE_addTemporalMarkerOffset(-0.9, () -> intakeMotor.setPower(0))
                    .splineToLinearHeading(new Pose2d(43, -42, Math.toRadians(180)), Math.toRadians(0))
                    //.waitSeconds(1)
                    .build();


            TrajectorySequence leftTraj2 = drive.trajectorySequenceBuilder(leftTraj1.end())
                    .waitSeconds(0.25)
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(1))
                    .strafeLeft(17)
                    .back(10)
                    .build();




            //middle
            TrajectorySequence middleTraj = drive.trajectorySequenceBuilder(new Pose2d(14.5, -61.5, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d (12, -33, Math.toRadians(-90)))
                    .back(-5)
                    .splineToLinearHeading(new Pose2d(40, -36, Math.toRadians(180)), Math.toRadians(0))
                    .build();

            TrajectorySequence middleTraj1 = drive.trajectorySequenceBuilder(middleTraj.end())
                    .waitSeconds(0.25)
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .forward(10)
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(0.7))
                    .splineToLinearHeading(new Pose2d(30, -60, Math.toRadians(180)), Math.toRadians(180))
                    .waitSeconds(0.25)
                    .lineToLinearHeading(new Pose2d(-13, -60, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-12.5, -10, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    .lineToLinearHeading(new Pose2d(-58, -11, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> intakeMotor.setPower(1))
                    .forward(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeMotor.setPower(0))
                    .lineToLinearHeading(new Pose2d(45, -10, Math.toRadians(180)))
                    .waitSeconds(1)
                    .splineToLinearHeading(new Pose2d(43, -42, Math.toRadians(180)), Math.toRadians(0))
                    //.waitSeconds(1)
                    .build();

            TrajectorySequence middleTraj2 = drive.trajectorySequenceBuilder(middleTraj1.end())
                    .waitSeconds(0.25)
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(1))
                    .strafeLeft(17)
                    .back(10)
                    .build();



            //right

            TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(new Pose2d(14.5, -61.5, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d (20, -40, Math.toRadians(-90)))
                    .back(-5)
                    .splineToLinearHeading(new Pose2d(43, -42, Math.toRadians(180)), Math.toRadians(0))
                    .build();

            TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(rightTraj.end())
                    .waitSeconds(0.25)
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .forward(10)
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(0.7))
                    //.splineToLinearHeading(new Pose2d(60, -64, Math.toRadians(180)), Math.toRadians(0))
                    //.build();


                    .splineToLinearHeading(new Pose2d(30, -60, Math.toRadians(90)), Math.toRadians(180))
                    .waitSeconds(0.25)
                    .lineToLinearHeading(new Pose2d(-13, -60, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(-12.5, -10, Math.toRadians(90)))
                    .waitSeconds(0.25)
                    .lineToLinearHeading(new Pose2d(-58, -11, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> intakeMotor.setPower(1))
                    .forward(2.5)
                    //.waitSeconds(2)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeMotor.setPower(0))
                    .lineToLinearHeading(new Pose2d(45, -10, Math.toRadians(180)))
                    .waitSeconds(1)
                    .splineToLinearHeading(new Pose2d(40, -28, Math.toRadians(180)), Math.toRadians(0))
                    .build();





            TrajectorySequence rightTraj2 = drive.trajectorySequenceBuilder(rightTraj1.end())
                    .waitSeconds(0.25)
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(1))
                    .strafeLeft(30)
                    .back(10)
                    .build();



            turnServo.setPosition(1);












            waitForStart();

            switch(PropHSVPipelineRed.getLocation()) {

                case LEFT:

                    drive.setPoseEstimate(new Pose2d(14.5, -61.5, Math.toRadians(-90)));

                    drive.followTrajectorySequence(leftTraj);


                    turnServo.setPosition(.85);
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

                    turnServo.setPosition(.85);
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

                    drive.followTrajectorySequence(leftTraj2);


                    break;


                case MIDDLE:

                    drive.setPoseEstimate(new Pose2d(14.5, -61.5, Math.toRadians(-90)));

                    drive.followTrajectorySequence(middleTraj);


                    turnServo.setPosition(.85);
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

                    turnServo.setPosition(.85);
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

                    drive.followTrajectorySequence(middleTraj2);


                    break;

                case RIGHT:
                    drive.setPoseEstimate(new Pose2d(14.5, -61.5, Math.toRadians(-90)));
                    drive.followTrajectorySequence(rightTraj);


                    turnServo.setPosition(.85);
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

                    turnServo.setPosition(.85);
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

                    drive.followTrajectorySequence(rightTraj2);

                    /*
                    drive.turn(Math.toRadians(-80));
                    drive.followTrajectory(forward);
                    drive.followTrajectory(backward);
                    drive.turn(Math.toRadians(-180));
                    drive.followTrajectory(rightboard);
                    drive.followTrajectory(boardstrafesmall);
                    drive.turn(Math.toRadians(70));

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

                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }


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

                     */
                    break;
            }
            camera.stopStreaming();

        }

    }
