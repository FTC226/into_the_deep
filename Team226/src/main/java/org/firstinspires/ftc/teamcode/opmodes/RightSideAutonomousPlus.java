package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subss.Wrist;


@Config
@Autonomous(name = "PlusRightInspiredAwardAutonomous", group = "Autonomous")
public class RightSideAutonomousPlus extends LinearOpMode {
    Wrist wrist = new Wrist(this);

    public class ArmSlidesClaw {
        private DcMotorEx arm, leftSlide, rightSlide;
        private Servo leftClaw, rightClaw, claw;

        private ElapsedTime timer = new ElapsedTime();

        public ArmSlidesClaw(HardwareMap hardwareMap) {

            wrist.init();

            arm = hardwareMap.get(DcMotorEx.class, "arm");
            leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
            rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

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

        public void moveSlidesVel(int targetSlides, double power) {
            leftSlide.setTargetPosition(targetSlides);
            rightSlide.setTargetPosition(targetSlides);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlide.setVelocity(power);
            rightSlide.setVelocity(power);
        }

        public void closeClaw() {
            claw.setPosition(0.7);
        }public void openClaw() {
            claw.setPosition(0);
        }

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(leftSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
        }

        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }

        public class PlaceFirstSpecimen implements Action {
            private int armPlacePosition = 1570;

            private int slidesPlace = 1000;

            private boolean isReset = false;

            private boolean isClipped = true;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }


                if (timer.seconds() <= 1.9) {
                    moveArm(armPlacePosition, 1);
                    if (isClipped) {
                        closeClaw();
                    }
                    wrist.PlaceSpecimen();

                    if (armReachedTarget(armPlacePosition, 20)) {
                        moveSlides(slidesPlace, 1);
                    }

                }

                if (timer.seconds() > 1.5) {
                    isClipped = false;
                    openClaw();
                }

                if (timer.seconds() > 1.9) {
                    moveSlides(0, 1);
                    moveArm(0, 1);
                }



                if (timer.seconds() > 2.3) {
                    isReset = false;
                    timer.reset();
                    return false;
                }

                return true;
            }
        }

        public Action placeFirstSpecimen() {
            return new RightSideAutonomousPlus.ArmSlidesClaw.PlaceFirstSpecimen();
        }

        public void resetTimer(){
            timer.reset();
        }



        public class PlaceSecondSpecimen implements Action {
            private int armPlacePosition = 1590;

            private int slidesPlace = 1000;

            private boolean isReset = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() <= 3.7) {
                    if (timer.seconds() > 1.8) {
                        moveArm(armPlacePosition, 1);
                    }
                    closeClaw();
                    wrist.PlaceSpecimen();

                    if (armReachedTarget(armPlacePosition, 20) && timer.seconds() > 2.7) {
                        moveSlides(slidesPlace, 1);
                    }

                }

                if (timer.seconds() > 3.2) {
                    openClaw();
                }

                if (timer.seconds() > 3.7) {
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
            return new RightSideAutonomousPlus.ArmSlidesClaw.PlaceSecondSpecimen();
        }

        public class PlaceOtherSpecimen implements Action {
            private int armPlacePosition = 1590;

            private int slidesPlace = 1000;

            private boolean isReset = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() <= 3.5) {
                    if (timer.seconds() > 1.8) {
                        moveArm(armPlacePosition, 1);
                    }
                    closeClaw();
                    wrist.PlaceSpecimen();

                    if (armReachedTarget(armPlacePosition, 20) && timer.seconds() > 2.4) {
                        moveSlides(slidesPlace, 1);
                    }

                }

                if (timer.seconds() > 3.5) {
                    openClaw();
                    moveSlides(0, 1);
                    if (slidesReachedTarget(0, 300)) {
                        moveArm(0, 1);
                    }
                }



                if (timer.seconds() > 3.8) {
                    isReset = false;
                    timer.reset();
                    return false;
                }

                return true;
            }
        }

        public Action placeOtherSpecimen() {
            return new RightSideAutonomousPlus.ArmSlidesClaw.PlaceOtherSpecimen();
        }

        public class PickUpSecondSpecimen implements Action {
            private int armPickUpPosition = 550;

            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    timer.reset();
                    isReset = true;
                }

                if (timer.seconds() <= 2.5) {
                    moveArm(armPickUpPosition, 1);
                    wrist.PickUpSpecimen();
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
            return new RightSideAutonomousPlus.ArmSlidesClaw.PickUpSecondSpecimen();
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
            return new RightSideAutonomousPlus.ArmSlidesClaw.Parking();
        }

        public class MoveSlidesDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveSlidesVel(0, 5500);
                moveArm(0, 1);
                return true;
            }
        }

        public Action moveSlidesDown() {
            return new RightSideAutonomousPlus.ArmSlidesClaw.MoveSlidesDown();
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(6, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        TrajectoryActionBuilder placeFirstSpecimen = drive.actionBuilder(initialPose)
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
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(

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
                )
        );

    }
}