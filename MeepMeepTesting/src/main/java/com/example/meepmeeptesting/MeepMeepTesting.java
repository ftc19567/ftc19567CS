package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(8, -67, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d (24, -40, Math.toRadians(-90)))
                                .splineToLinearHeading(new Pose2d(36, -47, Math.toRadians(0)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(47, -42, Math.toRadians(180)))
                                //.splineToLinearHeading(new Pose2d(51, -61, Math.toRadians(0)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(47, -61, Math.toRadians(180)))
                                .back (10)
                                //.splineToLinearHeading(new Pose2d(47, -42, Math.toRadians(-90)), Math.toRadians(-90))

                                //.splineToLinearHeading(new Pose2d(47, -42, Math.toRadians(-90)), Math.toRadians(-90))
                                //.turn(Math.toRadians(-90))
                                //.lineToLinearHeading(new Pose2d(47, -58, Math.toRadians(-180)))
                                //.forward(-15)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}