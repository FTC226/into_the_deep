package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class left_meepmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        Pose2d initialPose = new Pose2d(-37, -62, Math.toRadians(0));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        TrajectoryActionBuilder placeSample1 = myBot.getDrive().actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-50.2, -48.2), Math.toRadians(45));

        TrajectoryActionBuilder grabSample2 = placeSample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-46, -41), Math.toRadians(88));

        TrajectoryActionBuilder placeSample2 = grabSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-49.8, -47.8), Math.toRadians(45));

        TrajectoryActionBuilder grabSample3 = placeSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55.5 , -41), Math.toRadians(84));

        TrajectoryActionBuilder placeSample3 = grabSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50.2, -48.2), Math.toRadians(45));

        TrajectoryActionBuilder grabSample4 = placeSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-57, -43), Math.toRadians(108));

        TrajectoryActionBuilder placeSample4 = grabSample4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50.2, -48.2), Math.toRadians(45));

        TrajectoryActionBuilder grabSample5 = placeSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(107))
                .splineToLinearHeading(new Pose2d(-17.00, -10.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(0.65);

        TrajectoryActionBuilder grabSample6 = placeSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(107))
                .splineToLinearHeading(new Pose2d(-17, -15, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(150))
                .waitSeconds(0.65);


        TrajectoryActionBuilder placeSample5 = grabSample5.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52.2, -43.2, Math.toRadians(45)), Math.toRadians(270), new TranslationalVelConstraint(150));




        myBot.runAction(
                new SequentialAction(
                        placeSample1.build(),
                        grabSample2.build(),
                        placeSample2.build(),
                        grabSample3.build(),
                        placeSample3.build(),
                        grabSample4.build(),
                        placeSample4.build(),
                        grabSample5.build(),
                        placeSample5.build()

                )
        );




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}