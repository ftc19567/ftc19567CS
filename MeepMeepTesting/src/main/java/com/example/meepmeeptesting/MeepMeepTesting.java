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
                .setConstraints(63.1044330668311, 40, Math.toRadians(180), Math.toRadians(180), 14.26)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(9.5, 61.5, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d (22, 40, Math.toRadians(90)))
                                        .back(-5)
                                        .splineToLinearHeading(new Pose2d(40, 42, Math.toRadians(-180)), Math.toRadians(0))




        //.lineToLinearHeading(new Pose2d(47, -42, Math.toRadians(180)))
                                //.splineToLinearHeading(new Pose2d(51, -61, Math.toRadians(0)), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(47, -42, Math.toRadians(-90)), Math.toRadiamns(-90))

                                //.splineToLinearHeading(new Pose2d(47, -42, Math.toRadians(-90)), Math.toRadians(-90))
                                //.turn(Math.toRadians(-90))
                                //.lineToLinearHeading(new Pose2d(47, -58, Math.toRadians(-180)))
                                //.forward(-15)
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}