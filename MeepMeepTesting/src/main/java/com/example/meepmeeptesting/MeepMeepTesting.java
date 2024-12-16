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

        RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(200), Math.toRadians(200), 15)
                .build();

        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();



        TrajectoryActionBuilder placeSpecimenPath = right.getDrive().actionBuilder(new Pose2d(8, -62, Math.toRadians(270)))
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

        right.runAction(
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

        TrajectoryActionBuilder placeSpecimenPathLeft = left.getDrive().actionBuilder(new Pose2d(-14, -62, Math.toRadians(270)))
                .setTangent(Math.toRadians(70))
                .lineToY(-31)
                .waitSeconds(1); // Place specimen

        TrajectoryActionBuilder grabSample1Path = placeSpecimenPathLeft.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-40)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-47, Math.toRadians(90)) // Move to sample #1
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .waitSeconds(1); // Grab sample #1

        TrajectoryActionBuilder placeSample1Path = grabSample1Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(250))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #1
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample2Path = placeSample1Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(100))
                .lineToYLinearHeading(-40, Math.toRadians(90)) // Grab sample #2
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .waitSeconds(1);

//        TrajectoryActionBuilder grabSample2Path = placeSample1Path.endTrajectory().fresh()
//                .setTangent(Math.toRadians(115))
//                .lineToYLinearHeading(-48, Math.toRadians(90)) // Move to sample #2
//                .setTangent(Math.toRadians(90))
//                .lineToY(-35) // Grab sample #2
//                .waitSeconds(1);

        TrajectoryActionBuilder placeSample2Path = grabSample2Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #2
                .waitSeconds(1);

//        TrajectoryActionBuilder placeSample2Path = grabSample2Path.endTrajectory().fresh()
//                .lineToY(-48)
//                .setTangent(Math.toRadians(295))
//                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #2
//                .waitSeconds(1);

        TrajectoryActionBuilder grabSample3Path = placeSample2Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-26, Math.toRadians(180)) // Move to sample #3
                .setTangent(Math.toRadians(180))
                .lineToX(-58) // Grab sample #3
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample3Path = grabSample3Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #3
                .waitSeconds(1);

        TrajectoryActionBuilder parkAtSubmersiblePath = placeSample3Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .lineToYLinearHeading(-10, Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .lineToX(-23);
//                .setTangent(Math.toRadians(55))
//                .lineToXLinearHeading(-23, Math.toRadians(180)); // Park at submersible

        left.runAction(
                new SequentialAction(
                        placeSpecimenPathLeft.build(),
                        grabSample1Path.build(),
                        placeSample1Path.build(),
                        grabSample2Path.build(),
                        placeSample2Path.build(),
                        grabSample3Path.build(),
                        placeSample3Path.build(),
                        parkAtSubmersiblePath.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(right)
//                .addEntity(left)
//                .addEntity(testBot)
//                .addEntity(testBot1)
                .start();
    }
}