package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.PropHSVPipelineBlue;
import org.firstinspires.ftc.teamcode.auto.pipeline.PropHSVPipelineRed;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="RedAutoFarNOWHITE", group="NoWhiteSTRAFEISSUE")
public class RedAutoFarNoWhite extends LinearOpMode {
        private Servo turnServo;
        private Arm arm;
        private Intake intake;
        private DcMotor intakeMotor;
        OpenCvCamera camera;
        WebcamName webcam1;

        public double servoUpPos = 0.149;

        public int armUpPos = 1889;


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
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
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

            //right

            TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61.5, Math.toRadians(-90)))
                    .waitSeconds(5)


                    .lineToLinearHeading(new Pose2d (-37, -42, Math.toRadians(-90)))
                    .turn(Math.toRadians(-48))
                    .forward(-10)
                    .back(-10)
                    .turn(Math.toRadians(48))
                    .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(-90)))
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(15, -12))
                    .splineToLinearHeading(new Pose2d(43 ,-42.5, Math.toRadians(180)), Math.toRadians(-90))


                    .build();


            TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(rightTraj.end())
                    .waitSeconds(1.2)
                    .back(
                            10,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> arm.setPosition(1, 700))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> turnServo.setPosition(1))
                    .lineToLinearHeading(new Pose2d(47 ,-10, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .back(10)
                    .build();




            //middle
            TrajectorySequence middleTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61.5, Math.toRadians(-90)))
                    .waitSeconds(4)
                    .lineToLinearHeading(new Pose2d (-35.5, -32, Math.toRadians(-90)))
                    .back(-5)
                    .lineToLinearHeading(new Pose2d(-53, -35, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(-53, -8.5, Math.toRadians(-90)))
                    .turn(Math.toRadians(-90))
                    //.waitSeconds(6)
                    .lineTo(new Vector2d(15, -8.5))
                    .splineToLinearHeading(new Pose2d(43 ,-38, Math.toRadians(180)), Math.toRadians(-90))
                    .build();

            TrajectorySequence middleTraj1 = drive.trajectorySequenceBuilder(middleTraj.end())
                    .waitSeconds(1.2)
                    .back(
                            10,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .forward(12)
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> arm.setPosition(1, 700))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> turnServo.setPosition(1))
                    .lineToLinearHeading(new Pose2d(47 ,-12, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .back(10)
                    .build();



            //right

            TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61.5, Math.toRadians(-90)))
                    .waitSeconds(5)


                    .lineToLinearHeading(new Pose2d (-47, -36, Math.toRadians(-90)))
                    .forward(20)
                    .strafeRight(-10)
                    .lineToLinearHeading(new Pose2d (-35, -47, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(-90)))
                    .turn(Math.toRadians(-90))
                    //.splineToLinearHeading(new Pose2d(0 ,12, Math.toRadians(180)), Math.toRadians(180))
                    .lineTo(new Vector2d(15, -12))
                    .splineToLinearHeading(new Pose2d(43 ,-30, Math.toRadians(180)), Math.toRadians(-90))
                    .build();

            TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(leftTraj.end())
                    .waitSeconds(1.2)
                    .back(
                            10,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .forward(9)
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> arm.setPosition(1, 700))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> turnServo.setPosition(1))
                    .lineToLinearHeading(new Pose2d(47 ,-10, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .back(10)
                    .build();




            turnServo.setPosition(1);












            waitForStart();



            switch(PropHSVPipelineRed.getLocation()) {

                case LEFT:

                    drive.setPoseEstimate(new Pose2d(-35.5, -61.5, Math.toRadians(-90)));

                    drive.followTrajectorySequence(leftTraj);

                    //USED TO BE 1
                    turnServo.setPosition(.85);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(0.6, armUpPos);
                    try {
                        Thread.sleep(700);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(servoUpPos);


                    drive.followTrajectorySequence(leftTraj1);


                    break;


                case MIDDLE:

                    drive.setPoseEstimate(new Pose2d(-35.5, -61.5, Math.toRadians(-90)));

                    drive.followTrajectorySequence(middleTraj);

                    turnServo.setPosition(.85);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(0.6, armUpPos);
                    try {
                        Thread.sleep(700);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(servoUpPos);


                    drive.followTrajectorySequence(middleTraj1);


                    break;

                case RIGHT:
                    drive.setPoseEstimate(new Pose2d(-35.5, -61.5, Math.toRadians(-90)));
                    drive.followTrajectorySequence(rightTraj);

                    turnServo.setPosition(.85);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(0.6, armUpPos);
                    try {
                        Thread.sleep(700);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    turnServo.setPosition(servoUpPos);


                    drive.followTrajectorySequence(rightTraj1);

                    break;
            }
            camera.stopStreaming();

        }

    }
