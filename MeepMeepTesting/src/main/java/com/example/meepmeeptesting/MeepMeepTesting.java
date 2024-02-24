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
                                        .lineToLinearHeading(new Pose2d (-33, 32.7, Math.toRadians(90)))

                                        .back(-7)
                                        .turn(Math.toRadians(90))

                                        .lineToLinearHeading(new Pose2d(-60.5, 35, Math.toRadians(180)))
                                        .lineTo(new Vector2d(46, 35))

                                        //Arm Up




                                        .back(5)
                                        .waitSeconds(0.1)
                                        .strafeLeft(7)
                                        .lineTo(new Vector2d(25, 35))

                                        .lineTo(new Vector2d(-60.5, 35))

                                        //Arm Down


                                        .lineTo(new Vector2d(46, 35))

                                        //Arm Up


                                        .back(7
                                        )
                                        .lineTo(new Vector2d(-60.5, 35))

                                        //Arm Down

                                        .lineTo(new Vector2d(48, 35))

                                        //ARm Up


                                        .back(5
                                        )
                                        .forward(5)
                                        //Arm Down



                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}