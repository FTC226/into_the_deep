package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        TrajectoryActionBuilder placeSpecimenPath = rightNoSpline.getDrive().actionBuilder(new Pose2d(8, -62, Math.toRadians(270)))
                .setTangent(Math.toRadians(280))
                .lineToY(-31)
                .waitSeconds(1) //Placing Specimen
                ;


        TrajectoryActionBuilder firstSamplePath = placeSpecimenPath.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .lineToYLinearHeading(-40, Math.toRadians(270))
                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(35, Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-15,Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(45,Math.toRadians(0))
                ;

        TrajectoryActionBuilder pushFirstSample = firstSamplePath.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToYLinearHeading(-52, Math.toRadians(0))
                ;

        TrajectoryActionBuilder goBackToFirstSample = pushFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToYLinearHeading(-15, Math.toRadians(0))
                ;

        TrajectoryActionBuilder secondSamplePath = goBackToFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .lineToX(55)
                .setTangent(Math.toRadians(270))
                .lineToY(-52);


        TrajectoryActionBuilder pickUpSpecimenSecond = secondSamplePath.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(45, Math.toRadians(0))
                .setTangent(Math.toRadians(205))
                .lineToXLinearHeading(30, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(45,Math.toRadians(0))
                ;

        TrajectoryActionBuilder placeSpecimenSecond = pickUpSpecimenSecond.endTrajectory().fresh()
                .setTangent(Math.toRadians(148))
                .lineToYLinearHeading(-32, Math.toRadians(270))
                .waitSeconds(1) //Placing Specimen

                ;


        TrajectoryActionBuilder pickUpSpecimenThird = placeSpecimenSecond.endTrajectory().fresh()
                .setTangent(Math.toRadians(320))
                .lineToYLinearHeading(-58.5, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(44, Math.toRadians(0))
                ;

        TrajectoryActionBuilder placeSpecimenThird = pickUpSpecimenThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(149))
                .lineToYLinearHeading(-32, Math.toRadians(270))
                .waitSeconds(1) //Placing Specimen

                ;


        TrajectoryActionBuilder pickUpSpecimenFourth = placeSpecimenThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(320))
                .lineToYLinearHeading(-58.5, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(44, Math.toRadians(0))
                ;

        TrajectoryActionBuilder placeSpecimenFourth = pickUpSpecimenFourth.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .lineToYLinearHeading(-32, Math.toRadians(270))
                .waitSeconds(1) //Placing Specimen

                ;

        rightNoSpline.runAction(
                new SequentialAction(
                        placeSpecimenPath.build(),
                        firstSamplePath.build(),
                        pushFirstSample.build(),
                        goBackToFirstSample.build(),
                        secondSamplePath.build(),
                        pickUpSpecimenSecond.build(),
                        placeSpecimenSecond.build(),
                        pickUpSpecimenThird.build(),
                        placeSpecimenThird.build(),
                        pickUpSpecimenFourth.build(),
                        placeSpecimenFourth.build()
                )
        );


        RoadRunnerBotEntity rightWithSpline = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        TrajectoryActionBuilder placeFirstSpecimen = rightWithSpline.getDrive().actionBuilder(new Pose2d(8.00, -62.00, Math.toRadians(270.00)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-3.00, -34, Math.toRadians(270.00)), Math.toRadians(90.00))
                .waitSeconds(1)
                ;

        TrajectoryActionBuilder pushPath = placeFirstSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(24, -38.5), Math.toRadians(0.00))
                .splineToLinearHeading(new Pose2d(45, -15, Math.toRadians(0.00)), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(45, -52))
                .strafeToConstantHeading(new Vector2d(45, -15))
                .splineToLinearHeading(new Pose2d(55, -15, Math.toRadians(0.00)), Math.toRadians(270.00))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(55, -52))
                ;
        TrajectoryActionBuilder pickUpSecondSpecimen = pushPath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(35, -59.50, Math.toRadians(270.00)), Math.toRadians(270.00))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(35.00, -61.00))

                ;
        TrajectoryActionBuilder placeSecondSpecimen = pickUpSecondSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(0.00, -33.00))
                .waitSeconds(1)
                ;
        TrajectoryActionBuilder pickUpThirdSpecimen = placeSecondSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(35.00, -59.50))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(35.00, -61.00))
                ;

        TrajectoryActionBuilder placeThirdSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(2.00, -33.00))
                .waitSeconds(1)
                ;

        TrajectoryActionBuilder pickUpForthSpecimen = placeThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(35.00, -59.50))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(35.00, -61.00))
                ;

        TrajectoryActionBuilder placeForthSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(1.00, -33.00))
                .waitSeconds(1)

                ;



        rightWithSpline.runAction(
                new SequentialAction(
                        placeFirstSpecimen.build(),
                        pushPath.build(),
                        pickUpSecondSpecimen.build(),
                        placeSecondSpecimen.build(),
                        pickUpThirdSpecimen.build(),
                        placeThirdSpecimen.build(),
                        pickUpForthSpecimen.build(),
                        placeForthSpecimen.build()
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