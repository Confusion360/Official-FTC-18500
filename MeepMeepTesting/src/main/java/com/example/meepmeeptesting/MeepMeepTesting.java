package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d initialPose = new Pose2d(-38, -61, Math.toRadians(90));
        Pose2d basket = new Pose2d(-58.4, -55.48, Math.toRadians(-45));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action toBasket = myBot.getDrive().actionBuilder(initialPose)
                .strafeTo(new Vector2d(-40, -38))
                .turn(Math.toRadians(-45))
                //.lineToY(-56)
                .strafeTo(new Vector2d(-58.4, -55.48))
                .build();

        Action collectFirst = myBot.getDrive().actionBuilder(new Pose2d(-58.4, -55.48, Math.toRadians(45)))
                .strafeTo(new Vector2d(-35, -55.48))
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-35, -13))
                .strafeTo(new Vector2d(-47, -13))
                .build();

        Action toBasket2 = myBot.getDrive().actionBuilder(new Pose2d(-47, -13, Math.toRadians(90)))
                .strafeTo(new Vector2d(-40, -38))	//7.5 is half the robot
                .turn(Math.toRadians(-45))
                .strafeTo(basket.position)
                .build();

        SequentialAction stuff = new SequentialAction(
                toBasket,
                collectFirst,
                toBasket2
        );

        myBot.runAction(stuff);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}