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
                //Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(63.1044330668311, 40, Math.toRadians(180), Math.toRadians(180), 14.26)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61.5, Math.toRadians(-90)))
                                        .lineToLinearHeading(new Pose2d (-35.5, -33.5, Math.toRadians(-90)))
                                        .lineToLinearHeading(new Pose2d(-57, -35, Math.toRadians(180)))



                                        .lineToLinearHeading(new Pose2d(-57, -8.5, Math.toRadians(180)))
                                        .waitSeconds(15)
                                        .lineToLinearHeading(new Pose2d(40, -8.5, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(180)))
                                        .waitSeconds(0.25)

                                        .back(10)
                                       
                                        .forward(7)

                                        .strafeRight(24)
                                        .back(10)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}