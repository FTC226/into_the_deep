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

        Pose2d initialPose = new Pose2d(6, -62, Math.toRadians(270));


        RoadRunnerBotEntity rightWithSpline = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        TrajectoryActionBuilder placeFirstSpecimen = rightWithSpline.getDrive().actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-33, Math.toRadians(270))
                ;

        TrajectoryActionBuilder pickUpFirstSample = placeFirstSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(30, -40, Math.toRadians(42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder putFirstSample = pickUpFirstSample.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(-42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder pickUpSecondSample = putFirstSample.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(42, -40, Math.toRadians(42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder putSecondSample = pickUpSecondSample.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(40, -40, Math.toRadians(-42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder pickUpThirdSample = putSecondSample.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(50, -40, Math.toRadians(42.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder putThirdSample = pickUpThirdSample.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(42, -40, Math.toRadians(-42.00)), Math.toRadians(0.00))
                ;


        TrajectoryActionBuilder pickUpSecondSpecimen = putThirdSample.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(36, -40, Math.toRadians(270.00)), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(36, -55))
                .strafeToConstantHeading(new Vector2d(36, -59.5), new TranslationalVelConstraint(10))
                ;

        TrajectoryActionBuilder placeSecondSpecimen = pickUpSecondSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(3.5, -33), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder pickUpThirdSpecimen = placeSecondSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(36, -55), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(36, -59.5), new TranslationalVelConstraint(10))

                ;

        TrajectoryActionBuilder placeThirdSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(2.5, -32), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder pickUpForthSpecimen = placeThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(36, -55), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(36, -59.5), new TranslationalVelConstraint(10))
                ;

        TrajectoryActionBuilder placeForthSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(1.5, -32), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder pickUpFifthSpecimen = placeForthSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(36, -55  ), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(36, -59.5), new TranslationalVelConstraint(10))
                ;

        TrajectoryActionBuilder placeFifthSpecimen = pickUpFifthSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder park = placeThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(45.09, -60.00, Math.toRadians(320.00)), Math.toRadians(320.00), new TranslationalVelConstraint(300));
        ;


        rightWithSpline.runAction(
                new SequentialAction(

                        placeFirstSpecimen.build(),
                        pickUpFirstSample.build(),
                        putFirstSample.build(),
                        pickUpSecondSample.build(),
                        putSecondSample.build(),
                        pickUpThirdSample.build(),
                        putThirdSample.build(),
                        pickUpSecondSpecimen.build(),
                        placeSecondSpecimen.build(),
                        pickUpThirdSpecimen.build(),
                        placeThirdSpecimen.build(),
                        pickUpForthSpecimen.build(),
                        placeForthSpecimen.build(),
                        pickUpFifthSpecimen.build(),
                        placeFifthSpecimen.build(),
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
                .addEntity(rightWithSpline)
                .start();
    }
}