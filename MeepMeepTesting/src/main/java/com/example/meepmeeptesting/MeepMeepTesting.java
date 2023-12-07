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
                        drive.trajectorySequenceBuilder(new Pose2d(14.5, -61.5, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d (7, -34, Math.toRadians(-20)))
                                .back(-5)
                                .splineToLinearHeading(new Pose2d(40, -28, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .back(10)

                                .forward(10)

                                .splineToLinearHeading(new Pose2d(30, -60, Math.toRadians(180)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-13, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-12, -10, Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d(-58, -11, Math.toRadians(180)), Math.toRadians(90))

                                .forward(2.5)
                                .lineToLinearHeading(new Pose2d(45, -10, Math.toRadians(180)))
                                .waitSeconds(1)

                                .splineToLinearHeading(new Pose2d(43, -42, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(1)
                                .waitSeconds(0.5)
                                .back(10)

                                .forward(10)

                                .strafeLeft(17)
                                .back(10)



        //.lineToLinearHeading(new Pose2d(47, -42, Math.toRadians(180)))
                                //.splineToLinearHeading(new Pose2d(51, -61, Math.toRadians(0)), Math.toRadians(0))
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