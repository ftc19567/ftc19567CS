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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="BlueAutonFarONEWHITE")
public class BlueAutoFarOneWhite extends LinearOpMode {
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

            TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61.5, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d (-46.5, -33.5, Math.toRadians(-90)))
                    .back(-5)
                    .lineToLinearHeading(new Pose2d(-58, -35, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> intakeMotor.setPower(1))
                    .forward(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeMotor.setPower(0))
                    .lineToLinearHeading(new Pose2d(40, -35, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    .splineToLinearHeading(new Pose2d(40, -28, Math.toRadians(180)), Math.toRadians(0))
                    .waitSeconds(0.25)
                    .build();
            TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(leftTraj.end())
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(0.7))
                    .forward(7)
                    //.UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(0.7))
                    .strafeLeft(30)
                    .back(10)
                    .build();




            //middle
            TrajectorySequence middleTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61.5, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d (-35.5, 33.5, Math.toRadians(90)))
                    .back(-5)
                    .lineToLinearHeading(new Pose2d(-58, 35, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> intakeMotor.setPower(1))
                    .forward(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeMotor.setPower(0))
                    .lineToLinearHeading(new Pose2d(40, 35, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    .splineToLinearHeading(new Pose2d(40, 36, Math.toRadians(180)), Math.toRadians(0))
                    .waitSeconds(0.25)
                    .build();

            TrajectorySequence middleTraj1 = drive.trajectorySequenceBuilder(middleTraj.end())
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(0.7))
                    .forward(7)
                    //.UNSTABLE_addTemporalMarkerOffset(-0.9, () -> turnServo.setPosition(0.7))
                    .strafeLeft(22)
                    .back(10)
                    .build();



            //right

            TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61.5, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d (-45, 36, Math.toRadians(90)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, ()  -> turnServo.setPosition(0.67))
                    .back(-5)
                    .lineToLinearHeading(new Pose2d(-58, 35, Math.toRadians(180)))
                    .forward(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> intakeMotor.setPower(0.7))
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> intakeMotor.setPower(0))
                    .back(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> turnServo.setPosition(1))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> intakeMotor.setPower(-1))

                    .lineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> intakeMotor.setPower(0))
                    .splineToLinearHeading(new Pose2d(-40, 12, Math.toRadians(180)), Math.toRadians(0))
                    .lineTo(new Vector2d(20, 12))
                    .splineToLinearHeading(new Pose2d(43 ,29, Math.toRadians(180)), Math.toRadians(90))
                    .build();

            TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(rightTraj.end())
                    .waitSeconds(1)
                    .back(10)
                    .strafeRight(10)
                    .forward(9)
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> turnServo.setPosition(1))
                    .strafeLeft(30)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> arm.setPosition(1, 5))
                    .back(10)
                    .build();

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

            switch(PropHSVPipelineRed.getLocation()) {

                case LEFT:

                    drive.setPoseEstimate(new Pose2d(-35.5, -61.5, Math.toRadians(-90)));


                    break;


                case MIDDLE:

                    drive.setPoseEstimate(new Pose2d(-35.5, -61.5, Math.toRadians(-90)));


                    break;

                case RIGHT:
                    drive.setPoseEstimate(new Pose2d(-35.5, 61.5, Math.toRadians(90)));
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
                    turnServo.setPosition(0.18);


                    drive.followTrajectorySequence(rightTraj1);

                    break;
            }
            camera.stopStreaming();

        }

    }