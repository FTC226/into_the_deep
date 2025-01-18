package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(500), Math.toRadians(500), 15)
                .build();

        TrajectoryActionBuilder placeSample = myBot.getDrive().actionBuilder(new Pose2d(-37, -62, Math.toRadians(0)))
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(180));

        TrajectoryActionBuilder grabSample1 = placeSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(45))
                .lineToXLinearHeading(-48, Math.toRadians(90));

        TrajectoryActionBuilder placeSample1 = grabSample1.endTrajectory().fresh()
                .setTangent(Math.toRadians(225))
                .lineToXLinearHeading(-57, Math.toRadians(45));

        TrajectoryActionBuilder grabSample2 = placeSample1.endTrajectory().fresh()
                .setTangent(Math.toRadians(96))
                .lineToXLinearHeading(-58, Math.toRadians(90));

        TrajectoryActionBuilder placeSample2 = grabSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(276))
                .lineToXLinearHeading(-57, Math.toRadians(45));

        TrajectoryActionBuilder grabSample3 = placeSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .lineToYLinearHeading(-27, Math.toRadians(180));

        TrajectoryActionBuilder placeSample3 = grabSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(258))
                .lineToXLinearHeading(-58, Math.toRadians(45));

        TrajectoryActionBuilder parkAtSubmersible = placeSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25, -8, Math.toRadians(90)), Math.toRadians(0));
        ;

        myBot.runAction(
                new SequentialAction(
                        placeSample.build(),
                        grabSample1.build(),
                        placeSample1.build(),
                        grabSample2.build(),
                        placeSample2.build(),
                        grabSample3.build(),
                        placeSample3.build(),
                        parkAtSubmersible.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}