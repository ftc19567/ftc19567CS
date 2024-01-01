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
                                        //.lineToLinearHeading(new Pose2d (-24, 36, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d (-35, 47, Math.toRadians(90)))

                                        .lineToLinearHeading(new Pose2d(-35, 12, Math.toRadians(90)))
                                        .turn(Math.toRadians(90))
                                        //.splineToLinearHeading(new Pose2d(0 ,12, Math.toRadians(180)), Math.toRadians(180))
                                        .lineTo(new Vector2d(20, 12))
                                        .splineToLinearHeading(new Pose2d(43 ,28, Math.toRadians(180)), Math.toRadians(90))

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}