package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                //Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(63.1044330668311, 40, 6.891847157693078, 4.141592653589793, 12.86)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61.5, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d (-46.5, 33.5, Math.toRadians(90)))
                                        .back(-9)
                                        //.UNSTABLE_addDisplacementMarkerOffset(-0.8, () -> intakeServo.setPosition(.67))
                                        .lineToLinearHeading(new Pose2d(-58, 35, Math.toRadians(180))
                                             //   SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                              //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        //.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(1))
                                        .forward(1.5)
                                       // .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> intakeServo.setPosition(.78))
                                        .waitSeconds(1)
                                        //can maybe remove
                                        .back(4.2)
                                        .turn(Math.toRadians(90))
                                        .lineToLinearHeading(new Pose2d(-55.3, 12, Math.toRadians(-90)))
                                        .turn(Math.toRadians(-90))
                                       // .UNSTABLE_addTemporalMarkerOffset(-4, () -> intakeMotor.setPower(-0.6))
                                        .lineTo(new Vector2d(43, 12))
                                       // .UNSTABLE_addTemporalMarkerOffset(-1, () -> intakeMotor.setPower(0))

                                        .lineToLinearHeading(new Pose2d(43 ,29, Math.toRadians(180)))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}