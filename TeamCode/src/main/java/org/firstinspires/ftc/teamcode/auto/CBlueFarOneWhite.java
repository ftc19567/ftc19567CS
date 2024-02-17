package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.PropHSVPipelineBlue;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="BlueFarONEWHITE", group="Cleaned Code")
public class CBlueFarOneWhite extends LinearOpMode {
        private Servo turnServo;
        private Arm arm;
        private Intake intake;
        private DcMotor intakeMotor;

        private Servo intakeServo;
        OpenCvCamera camera;
        WebcamName webcam1;

        public double servoUpPos = 0.12;

        public int armUpPos = 1872;

        private void armUP() {
            turnServo.setPosition(.85);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            Arm.setPosition(0.6, armUpPos);
            try {
                Thread.sleep(700);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            turnServo.setPosition(servoUpPos);
        }

        private void armDown() {
            turnServo.setPosition(1);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            Arm.setPosition(1, 5);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            turnServo.setPosition(0.67);
        }

        @Override
        public void runOpMode() throws InterruptedException {

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            turnServo = hardwareMap.get(Servo.class, "turnServo");
            arm = new Arm(hardwareMap, telemetry);
            intake = new Intake(hardwareMap, telemetry);
            TrajectoriesBLUE trajInit = new TrajectoriesBLUE();

            PropHSVPipelineBlue pipeline = new PropHSVPipelineBlue(telemetry);


            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraMonitorViewId);
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

            //left

            TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61.5, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d (-46.5, 33.5, Math.toRadians(90)))
                    .back(-5)
                    .lineToLinearHeading(new Pose2d(-58, 35, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
                    .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(1))
                    .forward(2.3)
                    .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> intakeServo.setPosition(0.77))
                    .lineToLinearHeading(new Pose2d(20, 35, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> intakeMotor.setPower(0))
                    .splineTo(new Vector2d(40, 28), Math.toRadians(0))

                    .build();

            TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(rightTraj.end())
                    .waitSeconds(5)
                    .back(
                            12,
                            SampleMecanumDrive.getVelocityConstraint(6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .forward(
                            7,
                            SampleMecanumDrive.getVelocityConstraint(6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> turnServo.setPosition(0.7))

                    .lineToLinearHeading(new Pose2d(43, 12, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> Arm.setPosition(1, 5))
                    .build();




            //middle
            TrajectorySequence middleTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61.5, Math.toRadians(90)))
                    .waitSeconds(5)
                    .lineToLinearHeading(new Pose2d (-35.5, 32.7, Math.toRadians(90)))
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> turnServo.setPosition(0.67))
                    .back(-5)
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(-58, 35, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
                    .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(1))
                    .forward(2.3)
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> intakeServo.setPosition(0.78))
                    .lineTo(new Vector2d(20, 35))
                    .UNSTABLE_addTemporalMarkerOffset(-3, () -> intakeMotor.setPower(-0.6))
                    .splineTo(new Vector2d(40, 36), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-3, () -> intakeMotor.setPower(0))
                    .build();

            TrajectorySequence middleTraj1 = drive.trajectorySequenceBuilder(middleTraj.end())
                    .waitSeconds(1.5)
                    .back(
                            15,
                            SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .forward(
                            7,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> turnServo.setPosition(1))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> Arm.setPosition(1, 700))

                    .lineToLinearHeading(new Pose2d(43 ,12, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> Arm.setPosition(1, 5))
                    .back(10)
                    .build();



            //right


            /*
            TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61.5, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d (-35.5, -36, Math.toRadians(-90)))
                    .turn(Math.toRadians(-45))
                    .forward(-7)
                    .forward(5)
                    .lineToLinearHeading(new Pose2d(-58, -35, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> intakeMotor.setPower(1))
                    .forward(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeMotor.setPower(0))
                    .lineToLinearHeading(new Pose2d(40, -35, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    .splineToLinearHeading(new Pose2d(43, -42, Math.toRadians(180)), Math.toRadians(0))
                    .waitSeconds(0.25)
                    .build();

            TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(rightTraj.end())
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(0.7))
                    .forward(7)
                    //.UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(0.7))
                    .strafeLeft(17)
                    .back(10)
                    .build();

             */


            turnServo.setPosition(1);


            waitForStart();
            turnServo.setPosition(.67);
            drive.setPoseEstimate(new Pose2d(-35.5, 61.5, Math.toRadians(90)));

            switch(PropHSVPipelineBlue.getLocation()) {

                case LEFT:


                    telemetry.addData("fail? ", "fail");



                    break;


                case MIDDLE:
                    drive.followTrajectorySequence(trajInit.middleTraj);
                    armUP();
                    drive.followTrajectorySequence(trajInit.middleTraj1);
                    break;

                case RIGHT:
                    drive.followTrajectorySequence(trajInit.rightTraj);
                    armUP();
                    drive.followTrajectorySequence(trajInit.rightTraj1);
                    break;
            }
            camera.stopStreaming();

        }

    }
