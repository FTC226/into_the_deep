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

        public void wristDown() {
            leftClaw.setPosition(0.022);
            rightClaw.setPosition(0.762);
        }

        public void wristUp() {
            leftClaw.setPosition(0.25);
            rightClaw.setPosition(0.364);
        }

        public void wristPickUp() {
            leftClaw.setPosition(0.086);
            rightClaw.setPosition(0.554);
        }

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(leftSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
        }

        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }

        public class PlaceFirstSpecimen implements Action {
            private int armPlacePosition = 1500;

            private int slidesTargetPosition = 1000;

            private boolean wristPlaceSpecimen = false;
            private boolean resetWrist = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Move arm
                sleep(500);

                moveArm(armPlacePosition, 1);

                wristUp();


                // Once arm reached target, move slides
                if (armReachedTarget(armPlacePosition, 10)) {
                    moveSlides(slidesTargetPosition, 1);
                }

                // Once slide reached target, move wrist
                if (slidesReachedTarget(slidesTargetPosition, 5) && !wristPlaceSpecimen) {
                    wristPlaceSpecimen = true;
                    timer.reset();
                    wristDown();
                }

                claw.setPosition(0);


                moveSlides(0, 1);
                sleep(100);
                moveArm(0, 1);
                wristPickUp();



                return true;
            }
        }
        public Action placeFirstSpecimen() {
            return new RightSideAutonomous.ArmSlidesClaw.PlaceFirstSpecimen();
        }

        public class PickUpSecondSpecimen implements Action {
            private int armPickUpPosition = 750;

            private boolean wristPickUpSpecimen = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Move arm
                sleep(100);

                moveArm(armPickUpPosition, 1);

                wristPickUp();

                if (!wristPickUpSpecimen) {
                    wristPickUpSpecimen = true;
                    claw.setPosition(0);
                    sleep(200);
                }

                if (wristPickUpSpecimen) {
                    claw.setPosition(1);
                }
                return true;
            }
        }
        public Action pickUpSecondSpecimen() {
            return new RightSideAutonomous.ArmSlidesClaw.PickUpSecondSpecimen();
        }

        public class PickUpOtherSpecimen implements Action {
            private int armPickUpPosition = 750;

            private boolean wristPickUpSpecimen = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Move arm
                sleep(100);

                moveArm(armPickUpPosition, 1);

                wristPickUp();

                if (!wristPickUpSpecimen) {
                    wristPickUpSpecimen = true;
                    claw.setPosition(0);
                    sleep(200);
                }

                if (wristPickUpSpecimen) {
                    claw.setPosition(1);
                }
                return true;
            }
        }
        public Action pickUpOtherSpecimen() {
            return new RightSideAutonomous.ArmSlidesClaw.PickUpOtherSpecimen();
        }


        public class PlaceOtherSpecimen implements Action {
            private int armPlacePosition = 1500;

            private int armPickUpPosition = 750;


            private int slidesTargetPosition = 1000;

            private boolean wristPlaceSpecimen = false;
            private boolean resetWrist = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Move arm
                sleep(500);

                moveArm(armPlacePosition, 1);

                wristUp();


                // Once arm reached target, move slides
                if (armReachedTarget(armPlacePosition, 10)) {
                    moveSlides(slidesTargetPosition, 1);
                }

                // Once slide reached target, move wrist
                if (slidesReachedTarget(slidesTargetPosition, 5) && !wristPlaceSpecimen) {
                    wristPlaceSpecimen = true;
                    timer.reset();
                    wristDown();
                }

                claw.setPosition(0);


                moveSlides(0, 1);
                sleep(100);
                moveArm(armPickUpPosition, 1);
                wristPickUp();


                return true;
            }
        }
        public Action placeOtherSpecimen() {
            return new RightSideAutonomous.ArmSlidesClaw.PlaceOtherSpecimen();
        }
    }





    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(8, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        TrajectoryActionBuilder placeFirstSpecimen = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-3.00, -32, Math.toRadians(270.00)), Math.toRadians(90.00))
                .waitSeconds(1)
                ;

        TrajectoryActionBuilder pushPath = placeFirstSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(24, -38.5), Math.toRadians(0.00))
                .splineToLinearHeading(new Pose2d(45, -15, Math.toRadians(0.00)), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(45, -52))
                .strafeToConstantHeading(new Vector2d(45, -15))
                .splineToLinearHeading(new Pose2d(55, -15, Math.toRadians(0.00)), Math.toRadians(270.00))
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

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(

                        placeFirstSpecimen.build(),
                        pushPath.build(),
                        pickUpSecondSpecimen.build(),
                        placeSecondSpecimen.build(),
                        pickUpThirdSpecimen.build(),
                        placeThirdSpecimen.build(),
                        pickUpForthSpecimen.build(),
                        placeForthSpecimen.build()

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