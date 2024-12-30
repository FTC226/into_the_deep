package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity rightNoSpline = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(500), Math.toRadians(500), 15)
                .build();

        TrajectoryActionBuilder placeSpecimenPath = rightNoSpline.getDrive().actionBuilder(new Pose2d(8, -62, Math.toRadians(270)))
                .setTangent(Math.toRadians(105))
                .lineToYLinearHeading(-32, Math.toRadians(270))
                .waitSeconds(1)
                ;

        TrajectoryActionBuilder grabPlaceFirstSample = placeSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(32, -38.5), Math.toRadians(0.00))
                .turn(Math.toRadians(42))
                .waitSeconds(0.5)
                .turn(Math.toRadians(-84))
                .turn(Math.toRadians(42))
                ;
        TrajectoryActionBuilder grabPlaceSecondSample = grabPlaceFirstSample.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(48, -38.5), Math.toRadians(0.00))
                .turn(Math.toRadians(42))
                .waitSeconds(0.5)
                .turn(Math.toRadians(-84))
                .turn(Math.toRadians(42))
                ;

        TrajectoryActionBuilder grabPlaceThirdSample = grabPlaceSecondSample.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(53, -38.5), Math.toRadians(0.00))
                .turn(Math.toRadians(42))
                .waitSeconds(0.5)
                .turn(Math.toRadians(-84))
                ;
//        TrajectoryActionBuilder placeSecondSpecimen = pickUpSecondSpecimen.endTrajectory().fresh()
//                .setReversed(true)
//                .strafeToConstantHeading(new Vector2d(0.00, -33.00))
//                .waitSeconds(1)
//                ;
//        TrajectoryActionBuilder pickUpThirdSpecimen = placeSecondSpecimen.endTrajectory().fresh()
//                .setReversed(false)
//                .strafeToConstantHeading(new Vector2d(35.00, -59.50))
//                .waitSeconds(0.5)
//                .strafeToConstantHeading(new Vector2d(35.00, -61.00))
//                ;
//
//        TrajectoryActionBuilder placeThirdSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
//                .setReversed(true)
//                .strafeToConstantHeading(new Vector2d(2.00, -33.00))
//                .waitSeconds(1)
//                ;
//
//        TrajectoryActionBuilder pickUpForthSpecimen = placeThirdSpecimen.endTrajectory().fresh()
//                .setReversed(false)
//                .strafeToConstantHeading(new Vector2d(35.00, -59.50))
//                .waitSeconds(0.5)
//                .strafeToConstantHeading(new Vector2d(35.00, -61.00))
//                ;
//
//        TrajectoryActionBuilder placeForthSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
//                .setReversed(true)
//                .strafeToConstantHeading(new Vector2d(1.00, -33.00))
//                .waitSeconds(1)
//                ;
//
        rightNoSpline.runAction(
                new SequentialAction(
                        placeSpecimenPath.build(),
                        grabPlaceFirstSample.build(),
                        grabPlaceSecondSample.build(),
                        grabPlaceThirdSample.build()
//                        pickUpSecondSpecimen.build(),
//                        placeSecondSpecimen.build(),
//                        pickUpThirdSpecimen.build(),
//                        placeThirdSpecimen.build(),
//                        pickUpForthSpecimen.build(),
//                        placeForthSpecimen.build()
                )
        );


        RoadRunnerBotEntity rightWithSpline = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        TrajectoryActionBuilder placeFirstSpecimen = rightWithSpline.getDrive().actionBuilder(new Pose2d(8.00, -62.00, Math.toRadians(270.00)))
                .setTangent(Math.toRadians(105))
                .lineToYLinearHeading(-32, Math.toRadians(270))
                ;

        TrajectoryActionBuilder pushPath = placeFirstSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(18, -40), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(47, -15), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(47, -52))
                .strafeToLinearHeading(new Vector2d(47, -15), Math.toRadians(270.0))
                ;


        TrajectoryActionBuilder pickUpSecondSpecimen = pushPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(53, -15))
                .strafeToConstantHeading(new Vector2d(53, -61), new TranslationalVelConstraint(120))
                ;


        TrajectoryActionBuilder placeSecondSpecimen = pickUpSecondSpecimen.endTrajectory().fresh()
//                .strafeToConstantHeading(new Vector2d(8, -58), new TranslationalVelConstraint(120))
//                .strafeToConstantHeading(new Vector2d(-1, -32), new TranslationalVelConstraint(120))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90.00))
                ;



        TrajectoryActionBuilder pickUpThirdSpecimen = placeSecondSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35.80, -62), Math.toRadians(270.00))
                ;

        TrajectoryActionBuilder placeThirdSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(2, -32), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder pickUpForthSpecimen = placeThirdSpecimen.endTrajectory().fresh()
//                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(35.00, -59.50))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(35.00, -61.00))
                ;

        TrajectoryActionBuilder placeForthSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
//                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(1.00, -33.00))
                .waitSeconds(1)
                ;

        TrajectoryActionBuilder park = placeThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(27.09, -53.07, Math.toRadians(320.00)), Math.toRadians(320.00))
                ;
        Pose2d initialPose2 = new Pose2d(-0.22, -30.44, Math.toRadians(90.00));
        Pose2d secondPose = new Pose2d(32.89, -53.98, Math.toRadians(-39.76));
        Pose2d finalPose2 = new Pose2d(32.89, -53.98, Math.toRadians(-90));
        Pose2d lastPose = new Pose2d(32.89, -53.98, Math.toRadians(90));

        TrajectoryActionBuilder goToHumanPlayer = rightWithSpline.getDrive().actionBuilder(initialPose2)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(32.89, -53.98), Math.toRadians(-39.76))
                .waitSeconds(1)
                ;

        ;
        TrajectoryActionBuilder pickUpSpecimen = rightWithSpline.getDrive().actionBuilder(new Pose2d(32.89, -53.98, Math.toRadians(-39.76)))
                .setReversed(true)
                .turn(Math.toRadians(-90+39.76))
                .waitSeconds(1)
                ;

        TrajectoryActionBuilder goToSub = rightWithSpline.getDrive().actionBuilder(new Pose2d(32.89, -53.98, Math.toRadians(-90)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-0.22, -30.44),Math.toRadians(-90))
                .waitSeconds(1)
                ;
        TrajectoryActionBuilder turnToSub = rightWithSpline.getDrive().actionBuilder(new Pose2d(-0.22, -30.44,Math.toRadians(-90)))
                .turn(Math.toRadians(180))
                ;


        rightWithSpline.runAction(
                new SequentialAction(

                        placeFirstSpecimen.build(),
                        pushPath.build(),

                        pickUpSecondSpecimen.build(),
                        placeSecondSpecimen.build(),
                        pickUpThirdSpecimen.build(),
                        placeThirdSpecimen.build(),
                        park.build()

//                        pickUpForthSpecimen.build(),
//                        placeForthSpecimen.build()
//                        pickUpThirdSpecimen.build(),
//                        placeThirdSpecimen.build(),
//                        pickUpForthSpecimen.build(),
//                        placeForthSpecimen.build()

                )
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(rightNoSpline)
                .addEntity(rightWithSpline)
//                .addEntity(left)
//                .addEntity(testBot)
//                .addEntity(testBot1)
                .start();
    }
}