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
                                        .lineToLinearHeading(new Pose2d (-47, 36, Math.toRadians(90)))

                                        .lineToLinearHeading(new Pose2d (-35.5, 46, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-35, 11.4, Math.toRadians(90)))
                                        .turn(Math.toRadians(90))

                                        //intakepixels
                                        .lineTo(new Vector2d(-58, 11.4))


                                        .forward(2.5)

                                        .lineTo(new Vector2d(10, 11.4))

                                        //board dropoff
                                        .splineTo(new Vector2d(48, 28.5), Math.toRadians(0))

                                        .back(5)

                                        .waitSeconds(0.1)

                                        //moving to extra pixels
                                        .splineTo(new Vector2d(23, 58), Math.toRadians(180))
                                        .lineTo(new Vector2d(-33, 58))
                                        .splineTo(new Vector2d(-56, 35.3), Math.toRadians(180))

                                        //gettting pixels
                                        .forward(4.5)

                                        //moving to board
                                        .lineTo(new Vector2d(-58, 35.3))
                                        .splineTo(new Vector2d(-33, 58), Math.toRadians(0))
                                        .lineTo(new Vector2d(23, 58))
                                        .splineTo(new Vector2d(48, 30.5), Math.toRadians(0))
                                        .back(5)
                                        .forward(5)









                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}