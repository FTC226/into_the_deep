package org.firstinspires.ftc.teamcode.opmodes.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subss.Arm;
import org.firstinspires.ftc.teamcode.subss.Claw;
import org.firstinspires.ftc.teamcode.subss.Slides;
import org.firstinspires.ftc.teamcode.subss.Wrist;


@Autonomous(name = "RightSideAuto", group = "Autonomous")
public class RightSideAuto extends LinearOpMode {


    Arm arm = new Arm(this, 1);
    Slides slides = new Slides(this);
    Wrist wrist = new Wrist(this);
    Claw claw = new Claw(this);

    public class Subsystems {
        private ElapsedTime timer = new ElapsedTime();

        public Subsystems(HardwareMap hardwareMap) {
            arm.init();
            slides.init();
            wrist.init();
            claw.init();
//            forearm.init();
        }

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(slides.rightGetCurrentPosition() - targetSlides) < threshold && Math.abs(slides.rightGetCurrentPosition() - targetSlides) < threshold;
        }
        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }

        public void resetTimer() {timer.reset();}

        public class PlaceSpecimen1 implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() < 2.0) {
                    arm.moveUp();
//                    forearm.ReadyPlaceSpecimen();
                    claw.closeClaw();
                }

                if (timer.seconds() > 2.0) {
//                    forearm.PlaceSpecimen();
                    if (timer.seconds() > 2.2) {
                        claw.openClaw();
                    }
                }

                if (timer.seconds() > 2.3) {
//                    forearm.ReadyPickUpSample();
                    arm.moveDown();
                    slides.pickupSample();
                    wrist.PickUp45Right();

                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action placeSpecimen1() {
            return new RightSideAuto.Subsystems.PlaceSpecimen1();
        }

        public class PickUpSample1 implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() < 2.0) {
                    arm.moveDown();
//                    forearm.ReadyPickUpSample();
                    wrist.PickUp45Right();
                    claw.openClaw();
                }

                if (timer.seconds() > 2.0) {
//                    forearm.PickUpSample();
                    if (timer.seconds() > 2.2) {
                        claw.closeClaw();
                    }
                }

                if (timer.seconds() > 2.3) {
//                    forearm.ReadyPickUpSample();

                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action pickupSample1() {
            return new RightSideAuto.Subsystems.PickUpSample1();
        }

        public class DropSample1 implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() > 1) {
                    claw.openClaw();
                }

                if (timer.seconds() > 2.3) {
                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action dropsample1() {
            return new RightSideAuto.Subsystems.DropSample1();
        }

        public class PickUpSample2 implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() < 2.0) {
                    arm.moveDown();
//                    forearm.ReadyPickUpSample();
                    wrist.PickUp45Right();
                    claw.openClaw();
                }

                if (timer.seconds() > 2.0) {
//                    forearm.PickUpSample();
                    if (timer.seconds() > 2.2) {
                        claw.closeClaw();
                    }
                }

                if (timer.seconds() > 2.3) {
//                    forearm.ReadyPickUpSample();

                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action pickupSample2() {
            return new RightSideAuto.Subsystems.PickUpSample2();
        }

        public class DropSample2 implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() > 1) {
                    claw.openClaw();
                }

                if (timer.seconds() > 2.3) {
                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action dropsample2() {
            return new RightSideAuto.Subsystems.DropSample2();
        }

        public class PickUpSample3 implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() < 2.0) {
                    arm.moveDown();
//                    forearm.ReadyPickUpSample();
                    wrist.PickUp45Right();
                    claw.openClaw();
                }

                if (timer.seconds() > 2.0) {
                    if (timer.seconds() > 2.2) {
                        claw.closeClaw();
                    }
                }

                if (timer.seconds() > 2.3) {

                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action pickupSample3() {
            return new RightSideAuto.Subsystems.PickUpSample3();
        }

        public class DropSample3 implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() > 1) {
                    claw.openClaw();
                }

                if (timer.seconds() > 2.3) {
                    arm.moveUp();
                    slides.moveToResetPos();

                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action dropsample3() {
            return new RightSideAuto.Subsystems.DropSample3();
        }

        public class PickUpSpecimen2 implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() < 2.0) {
                    wrist.PickUpSpecimen();
                }

                if (timer.seconds() > 2) {
                    claw.closeClaw();
                }

                if (timer.seconds() > 2.3) {
                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action pickupSpecimen2() {
            return new RightSideAuto.Subsystems.PickUpSpecimen2();
        }

        public class PickUpSpecimen implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() < 2.0) {
                    wrist.PickUpSpecimen();
                }

                if (timer.seconds() > 2) {
                    claw.closeClaw();
                }

                if (timer.seconds() > 2.3) {
                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action pickupSpecimen() {
            return new RightSideAuto.Subsystems.PickUpSpecimen();
        }

        public class PlaceSpecimen implements Action {
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() < 2.0) {
                    arm.moveUp();
                    claw.closeClaw();
                }

                if (timer.seconds() > 2.0) {
                    if (timer.seconds() > 2.2) {
                        claw.openClaw();
                    }
                }

                if (timer.seconds() > 2.3) {
                    isReset = false;
                    timer.reset();
                    return false;
                }
                return false;
            }
        }

        public Action placeSpecimen() {
            return new RightSideAuto.Subsystems.PlaceSpecimen();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Subsystems subsystems = new Subsystems(hardwareMap);



        TrajectoryActionBuilder placeSpecimen1 = drive.actionBuilder(initialPose)
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


        //initialize
        arm.moveUp();
        claw.closeClaw();



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
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
//                        new ParallelAction(
//                                placeSpecimen1.build(),
//                                subsystems.placeSpecimen1()
//                        ),
//
//                        new ParallelAction(
//                                pickUpSample1.build(),
//                                subsystems.pickupSample1()
//                        ),
//
//                        new ParallelAction(
//                                dropSample1.build(),
//                                subsystems.dropsample1()
//                        ),
//
//                        new ParallelAction(
//                                pickUpSample2.build(),
//                                subsystems.pickupSample2()
//                        ),
//
//                        new ParallelAction(
//                                dropSample2.build(),
//                                subsystems.dropsample2()
//                        ),
//
//                        new ParallelAction(
//                                pickUpSample3.build(),
//                                subsystems.pickupSample3()
//                        ),
//
//                        new ParallelAction(
//                                dropSample3.build(),
//                                subsystems.dropsample3()
//                        ),
//
//                        new ParallelAction(
//                                pickUpSpecimen1.build(),
//                                subsystems.pickupSpecimen2() // pickup the first specimen on the wall
//                        ),
//
//                        new ParallelAction(
//                                placeSpecimen2.build(),
//                                subsystems.placeSpecimen() // place the second specimen
//                        ),
//
//                        new ParallelAction(
//                                pickUpSpecimen2.build(),
//                                subsystems.pickupSpecimen() // pickup the second specimen on the wall
//                        ),
//
//                        new ParallelAction(
//                                placeSpecimen3.build(),
//                                subsystems.placeSpecimen()  // place the third specimen
//                        ),
//
//                        new ParallelAction(
//                                pickUpSpecimen3.build(),
//                                subsystems.pickupSpecimen() // pickup the third specimen on the wall
//                        ),
//
//                        new ParallelAction(
//                                placeSpecimen4.build(),
//                                subsystems.placeSpecimen()  // place the fourth specimen
//                        ),
//
//                        new ParallelAction(
//                                pickUpSpecimen4.build(),
//                                subsystems.pickupSpecimen() // pickup the fourth specimen on the wall
//                        ),
//
//                        new ParallelAction(
//                                placeSpecimen5.build(),
//                                subsystems.placeSpecimen()  // place the five specimen
//                        ),
//
//                        new ParallelAction(
//                                park.build()
//                        )



                )
        );
    }
}
