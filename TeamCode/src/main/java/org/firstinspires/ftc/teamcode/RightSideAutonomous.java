package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

@Config
@Autonomous(name = "RightSideAutonomous", group = "Autonomous")
public class RightSideAutonomous extends LinearOpMode {

    public class ArmSlidesClaw {
        private DcMotor arm, leftSlide, rightSlide;
        private Servo leftClaw, rightClaw, claw;

        private ElapsedTime timer = new ElapsedTime();

        public ArmSlidesClaw(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotor.class, "arm");
            leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
            rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

            leftClaw = hardwareMap.get(Servo.class, "leftServo");
            rightClaw = hardwareMap.get(Servo.class, "rightServo");
            claw = hardwareMap.get(Servo.class, "clawServo");

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            arm.setDirection(DcMotor.Direction.REVERSE);
            leftSlide.setDirection(DcMotor.Direction.FORWARD);
            rightSlide.setDirection(DcMotor.Direction.REVERSE);
        }

        public void moveArm(int targetArm, double power) {
            arm.setTargetPosition(targetArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);
        }

        public void moveSlides(int targetSlides, double power) {
            leftSlide.setTargetPosition(targetSlides);
            rightSlide.setTargetPosition(targetSlides);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlide.setPower(power);
            rightSlide.setPower(power);
        }

        public void wristUp() {
            leftClaw.setPosition(0.944);
            rightClaw.setPosition(0.33);
        }

        public void wristDown() {
            leftClaw.setPosition(0.364);
            rightClaw.setPosition(0.664);
        }

        public void wristPlaceSpecimen() {
            leftClaw.setPosition(0.666);
            rightClaw.setPosition(0.396);
        }

        public void wristPickUp() {
            leftClaw.setPosition(0.5578);
            rightClaw.setPosition(0.5356);
        }

        public void closeClaw() {
            claw.setPosition(1);
        }public void openClaw() {
            claw.setPosition(0.2);
        }

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(leftSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
        }

        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }

        public class PlaceFirstSpecimen implements Action {
            private int armPlacePosition = 1570;

            private int slidesPlace = 900;

            private boolean isReset = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (timer.seconds() <= 2.8) {
                    moveArm(armPlacePosition, 1);
                    closeClaw();
                    wristPlaceSpecimen();

                    if (armReachedTarget(armPlacePosition, 20)) {
                        moveSlides(slidesPlace, 1);
                    }

                }

                if (timer.seconds() > 2.9) {
                    openClaw();
                    moveSlides(0, 1);
                    moveArm(0, 1);
                }



                if (timer.seconds() > 3) {
                    isReset = false;
                    timer.reset();
                    return false;
                }

                return true;
            }
        }

        public Action placeFirstSpecimen() {
            return new RightSideAutonomous.ArmSlidesClaw.PlaceFirstSpecimen();
        }

        public void resetTimer(){
            timer.reset();
        }



        public class PlaceSecondSpecimen implements Action {
            private int armPlacePosition = 1590;

            private int slidesPlace = 900;

            private boolean isReset = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() <= 3.9) {
                    if (timer.seconds() > 1.8) {
                        moveArm(armPlacePosition, 1);
                    }
                    closeClaw();
                    wristPlaceSpecimen();

                    if (armReachedTarget(armPlacePosition, 20) && timer.seconds() > 2.5) {
                        moveSlides(slidesPlace, 1);
                    }

                }

                if (timer.seconds() > 3.9) {
                    openClaw();
                    moveSlides(0, 1);
                    if (slidesReachedTarget(0, 300)) {
                        moveArm(0, 1);
                    }
                }



                if (timer.seconds() > 4) {
                    isReset = false;
                    timer.reset();
                    return false;
                }

                return true;
            }
        }

        public Action placeSecondSpecimen() {
            return new RightSideAutonomous.ArmSlidesClaw.PlaceSecondSpecimen();
        }

        public class PlaceOtherSpecimen implements Action {
            private int armPlacePosition = 1590;

            private int slidesPlace = 900;

            private boolean isReset = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() <= 3.2) {
                    if (timer.seconds() > 1.8) {
                        moveArm(armPlacePosition, 1);
                    }
                    closeClaw();
                    wristPlaceSpecimen();

                    if (armReachedTarget(armPlacePosition, 20) && timer.seconds() > 2.2) {
                        moveSlides(slidesPlace, 1);
                    }

                }

                if (timer.seconds() > 3.2) {
                    openClaw();
                    moveSlides(0, 1);
                    if (timer.seconds() > 3) {
                        moveArm(0, 1);

                    }
                }



                if (timer.seconds() > 3.6) {
                    isReset = false;
                    timer.reset();
                    return false;
                }

                return true;
            }
        }

        public Action placeOtherSpecimen() {
            return new RightSideAutonomous.ArmSlidesClaw.PlaceOtherSpecimen();
        }

        public class PickUpSecondSpecimen implements Action {
            private int armPickUpPosition = 650;

            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    timer.reset();
                    isReset = true;
                }

                if (timer.seconds() <= 2.5) {
                    moveArm(armPickUpPosition, 1);
                    wristPickUp();
                    openClaw();
                }

                if (timer.seconds() > 2.3) {
                    closeClaw();
                }

                if (timer.seconds() > 3) {
                    isReset = false;
                    return false;
                }

                return true;
            }
        }

        public Action pickUpSecondSpecimen() {
            return new RightSideAutonomous.ArmSlidesClaw.PickUpSecondSpecimen();
        }


        public class Parking implements Action {

            private int slidesParking = 2100;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveSlides(slidesParking, 1);
                return true;
            }
        }

        public Action parking() {
            return new RightSideAutonomous.ArmSlidesClaw.Parking();
        }

    }





    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(8, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        TrajectoryActionBuilder placeFirstSpecimen = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(106))
                .lineToYLinearHeading(-32, Math.toRadians(270))
                ;

        TrajectoryActionBuilder pushPath = placeFirstSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(22, -40), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(44, -18), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(44, -52))
                .strafeToLinearHeading(new Vector2d(47, -15), Math.toRadians(270.0))

                ;


        TrajectoryActionBuilder pickUpSecondSpecimen = pushPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(53, -18))
                .strafeToConstantHeading(new Vector2d(53, -61))
                ;


        TrajectoryActionBuilder placeSecondSpecimen = pickUpSecondSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(1, -35), Math.toRadians(90.00))
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
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35.80, -62), Math.toRadians(270.00))
                ;

        TrajectoryActionBuilder placeForthSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(3, -32), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder park = placeThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(27.09, -53.07, Math.toRadians(320.00)), Math.toRadians(320.00))
                ;

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                placeFirstSpecimen.build(),
                                armslidesclaw.placeFirstSpecimen()
                        ),

                        pushPath.build(),

                        new ParallelAction(
                                pickUpSecondSpecimen.build(),
                                armslidesclaw.pickUpSecondSpecimen()
                        ),
////
                        new ParallelAction(
                                placeSecondSpecimen.build(),
                                armslidesclaw.placeSecondSpecimen()
                        ),

                        new ParallelAction(
                                pickUpThirdSpecimen.build(),
                                armslidesclaw.pickUpSecondSpecimen()

                        ),

                        new ParallelAction(
                                placeThirdSpecimen.build(),
                                armslidesclaw.placeOtherSpecimen()
                        ),

                        new ParallelAction(
                                pickUpForthSpecimen.build(),
                                armslidesclaw.pickUpSecondSpecimen()

                        ),

                        new ParallelAction(
                                placeForthSpecimen.build(),
                                armslidesclaw.placeOtherSpecimen()
                        ),

                        new ParallelAction(
                                park.build(),
                                armslidesclaw.parking()
                        )

//                        armslidesclaw.resetSlides(),
//                        armslidesclaw.resetArm(),




//                        grabPlaceFirstSample.build(),
//                        grabPlaceSecondSample.build(),
//                        grabPlaceThirdSample.build()


//                        pushPath.build()
//                        pickUpSecondSpecimen.build(),
//                        placeSecondSpecimen.build(),
//                        pickUpThirdSpecimen.build(),
//                        placeThirdSpecimen.build(),
//                        pickUpForthSpecimen.build(),
//                        placeForthSpecimen.build()

//                        new ParallelAction(
//                                placeFirstSpecimen.build(),
//                                armslidesclaw.placeFirstSpecimen()
//                        )

//                        new ParallelAction(
//                                pushPath.build()
//                        ),
//
//                        new ParallelAction(
//                                pickUpSecondSpecimen.build(),
//                                armslidesclaw.pickUpSecondSpecimen()
//                        ),
//
//                        new ParallelAction(
//                                placeSecondSpecimen.build(),
//                                armslidesclaw.placeOtherSpecimen()
//                        ),
//
//                        new ParallelAction(
//                                pickUpThirdSpecimen.build(),
//                                armslidesclaw.pickUpOtherSpecimen()
//                        ),
//
//                        new ParallelAction(
//                                placeThirdSpecimen.build(),
//                                armslidesclaw.placeOtherSpecimen()
//                        ),
//                        new ParallelAction(
//                                pickUpForthSpecimen.build(),
//                                armslidesclaw.pickUpOtherSpecimen()
//                        ),
//
//                        new ParallelAction(
//                                placeForthSpecimen.build(),
//                                armslidesclaw.placeOtherSpecimen()
//                        )
                )
        );

    }
}