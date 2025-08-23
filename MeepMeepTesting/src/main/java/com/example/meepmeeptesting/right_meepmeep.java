package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class right_meepmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(640);
        System.setProperty("sun.java2d.opengl", "true");

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(270));

        TrajectoryActionBuilder placeSpecimen1 = myBot.getDrive().actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(10, -35))
                ;

        TrajectoryActionBuilder pickUpSample1 = placeSpecimen1.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(32, -40, Math.toRadians(42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder dropSample1 = pickUpSample1.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(42, -40, Math.toRadians(-42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder pickUpSample2 = dropSample1.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(42, -40, Math.toRadians(42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder dropSample2 = pickUpSample2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(38, -40, Math.toRadians(-42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder pickUpSample3 = dropSample2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(52, -40, Math.toRadians(42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder dropSample3 = pickUpSample3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(-42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder pickUpSpecimen1 = dropSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -60), Math.toRadians(270))
                ;

        TrajectoryActionBuilder placeSpecimen2 = pickUpSpecimen1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(8, -35))
                ;

        TrajectoryActionBuilder pickUpSpecimen2 = placeSpecimen2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -50), Math.toRadians(270))
//                .waitSeconds(0.5) //important
                .strafeToLinearHeading(new Vector2d(35, -60), Math.toRadians(270))
                ;

        TrajectoryActionBuilder placeSpecimen3 = pickUpSpecimen2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(8, -35))
                ;
        TrajectoryActionBuilder pickUpSpecimen3 = placeSpecimen3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -50), Math.toRadians(270))
//                .waitSeconds(0.5) //important
                .strafeToLinearHeading(new Vector2d(35, -60), Math.toRadians(270))
                ;

        TrajectoryActionBuilder placeSpecimen4 = pickUpSpecimen3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(6, -35))
                ;

        TrajectoryActionBuilder pickUpSpecimen4 = placeSpecimen4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -50), Math.toRadians(270))
//                .waitSeconds(0.5) //important
                .strafeToLinearHeading(new Vector2d(35, -60), Math.toRadians(270))
                ;

        TrajectoryActionBuilder placeSpecimen5 = pickUpSpecimen4.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(4, -35))
                ;

        TrajectoryActionBuilder park = placeSpecimen5.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(40, -60))
                ;


        myBot.runAction(
                new SequentialAction(
                        placeSpecimen1.build(),
                        pickUpSample1.build(),
                        dropSample1.build(),
                        pickUpSample2.build(),
                        dropSample2.build(),
                        pickUpSample3.build(),
                        dropSample3.build(),
                        pickUpSpecimen1.build(),
                        placeSpecimen2.build(),
                        pickUpSpecimen2.build(),
                        placeSpecimen3.build(),
                        pickUpSpecimen3.build(),
                        placeSpecimen4.build(),
                        pickUpSpecimen4.build(),
                        placeSpecimen5.build(),
                        park.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}