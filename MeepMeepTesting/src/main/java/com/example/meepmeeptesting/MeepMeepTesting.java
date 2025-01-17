package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.UMathKt;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(200), Math.toRadians(200), 17)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-11.1, 60, Math.toRadians(270)))

                .strafeTo(new Vector2d(-11.1, 28.9))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-11.1, 45.9))


                .setTangent(9)
                .splineToLinearHeading(new Pose2d(-46.1, 13.8,Math.toRadians(90)), Math.PI / 2)
                .strafeTo(new Vector2d(-46.1, 60.5))
                .waitSeconds(0.8)
                .setTangent(1)
                .splineToSplineHeading(new Pose2d(-6.1, 36.9,Math.toRadians(270)), Math.PI / 10)
                .strafeTo(new Vector2d(-6.1, 28.9))
                .waitSeconds(0.8)

                .strafeTo(new Vector2d(-6.1, 40.9))

                .splineToLinearHeading(new Pose2d(-58.1, 13.9,Math.toRadians(90)), Math.PI / 2)

                .strafeTo(new Vector2d(-58.1, 56))

                .splineToSplineHeading(new Pose2d(-1.1, 36.9,Math.toRadians(270)), Math.PI / 10)

                .strafeTo(new Vector2d(-1.1, 28.9))
                .waitSeconds(0.8)
                .strafeTo(new Vector2d(-1.1, 45.9))

                .splineToSplineHeading(new Pose2d(-37.1, 57.9,Math.toRadians(90)), Math.PI / 1)
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(5.1, 36.9,Math.toRadians(270)), Math.PI / 10)
                .strafeTo(new Vector2d(5.1, 28.9))










//second specimen







                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}



//.strafeTo (new Vector2d(36.9,12.8))
        //.strafeTo(new Vector2d(41.9,12.8))
        //.strafeTo(new Vector2d(41.9,69.2))
        //.splineToLinearHeading(new Pose2d(55.5, 20.8, Math.toRadians(270)), Math.PI / 2)
       // .strafeTo(new Vector2d(55.9,69.2))

        //.splineToLinearHeading(new Pose2d(61.5, 20.8, Math.toRadians(270)), Math.PI / 2)
