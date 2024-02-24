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


@Autonomous(name="BlueAutonFarONEWHITE", group="OneWhiteUNTESTED")
public class BlueAutoFarOneWhite extends LinearOpMode {
        private Servo turnServo;
        private Arm arm;
        private Intake intake;
        private DcMotor intakeMotor;

        private Servo intakeServo;
        OpenCvCamera camera;
        WebcamName webcam1;

        public double servoUpPos = 0.12;

        public int armUpPos = 1860;

        public double intakeUpPos = 0.766;

        public double intakeOnePixelPos = .781;



        private void armUP() {
            turnServo.setPosition(1);
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
        }


        @Override
        public void runOpMode() throws InterruptedException {

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            turnServo = hardwareMap.get(Servo.class, "turnServo");
            arm = new Arm(hardwareMap, telemetry);

            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

            intakeServo = hardwareMap.get(Servo.class, "intakeServo");


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

            TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61.5, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d (-47, 36, Math.toRadians(90)))
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> intakeServo.setPosition(intakeUpPos))
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> turnServo.setPosition(0.67))
                    .lineToLinearHeading(new Pose2d (-35.5, 46, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(-35, 11.4, Math.toRadians(90)))
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(-58, 11.4))
                    .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(1))
                    .forward(2.5)
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> intakeServo.setPosition(intakeOnePixelPos))
                    .lineTo(new Vector2d(10, 11.4))
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> intakeMotor.setPower(-0.6))
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> intakeMotor.setPower(0))
                    .splineTo(new Vector2d(41, 28.5), Math.toRadians(0))
                    .build();

            TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(rightTraj.end())
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
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> arm.setPosition(1, 700))

                    .lineToLinearHeading(new Pose2d(43 ,12, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .back(10)
                    .build();




            //middle
            TrajectorySequence middleTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61.5, Math.toRadians(90)))
                    //.waitSeconds(5)
                    .lineToLinearHeading(new Pose2d (-35.5, 32.7, Math.toRadians(90)))
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> turnServo.setPosition(0.67))
                    .back(-5)
                    .turn(Math.toRadians(90))
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> intakeServo.setPosition(intakeUpPos))
                    .lineToLinearHeading(new Pose2d(-58, 35, Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
                    .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(1))
                    .forward(2.5)
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> intakeServo.setPosition(intakeOnePixelPos))
                    .waitSeconds(1)
                    //can maybe remove
                    .lineTo(new Vector2d(40, 35))
                    .UNSTABLE_addTemporalMarkerOffset(-4, () -> intakeMotor.setPower(-0.6))
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> intakeMotor.setPower(0))
                    .build();

            TrajectorySequence middleTraj1 = drive.trajectorySequenceBuilder(middleTraj.end())
                    .waitSeconds(1.5)
                    .back(
                            13,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .strafeLeft(
                            7,
                            SampleMecanumDrive.getVelocityConstraint(6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .forward(
                            7,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> turnServo.setPosition(1))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> arm.setPosition(1, 700))

                    .lineToLinearHeading(new Pose2d(43 ,12, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .back(10)
                    .build();

//left
            TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61.5, Math.toRadians(90)))

                    .lineToLinearHeading(new Pose2d (-37, 42, Math.toRadians(90)))
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> intakeServo.setPosition(intakeUpPos))
                    .turn(Math.toRadians(48))
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> turnServo.setPosition(0.67))
                    .forward(-10)
                    .lineToLinearHeading(new Pose2d(-58, 35.5, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(1))
                    .forward(2.5)
                    .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> intakeServo.setPosition(intakeOnePixelPos))
                    .waitSeconds(1)
                    .back(5)
                    //.UNSTABLE_addDisplacementMarkerOffset(0, () -> intakeServo.setPosition(0.782))
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> intakeMotor.setPower(-0.6))
                    .lineToLinearHeading(new Pose2d (-37, 60, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> intakeMotor.setPower(0))
                    .lineTo(new Vector2d(20, 60))

                    .splineTo(new Vector2d(41, 41.2), Math.toRadians(0))
                    .build();


            TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(leftTraj.end())
                    .waitSeconds(1.5)

                    .back(
                            13,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .strafeLeft(
                           7,
                          SampleMecanumDrive.getVelocityConstraint(6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .forward(
                            7,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> turnServo.setPosition(1))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> arm.setPosition(1, 700))

                    .lineToLinearHeading(new Pose2d(43 ,12, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .back(10)
                    .build();



            turnServo.setPosition(.67);

            waitForStart();

            switch(PropHSVPipelineBlue.getLocation()) {

                case LEFT:

                    drive.setPoseEstimate(new Pose2d(-35.5, 61.5, Math.toRadians(90)));
                    drive.followTrajectorySequence(leftTraj);

                    armUP();

                    drive.followTrajectorySequence(leftTraj1);


                    break;


                case MIDDLE:

                    drive.setPoseEstimate(new Pose2d(-35.5, 61.5, Math.toRadians(90)));
                    drive.followTrajectorySequence(middleTraj);

                    armUP();

                    drive.followTrajectorySequence(middleTraj1);


                    break;

                case RIGHT:
                    drive.setPoseEstimate(new Pose2d(-35.5, 61.5, Math.toRadians(90)));
                    drive.followTrajectorySequence(rightTraj);

                    armUP();

                    drive.followTrajectorySequence(rightTraj1);

                    break;
            }
            camera.stopStreaming();

        }

    }
