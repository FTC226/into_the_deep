package org.firstinspires.ftc.teamcode.opmode.auto;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.auto.mecanumdrive.RightMecanumDrive;


@Config
@Autonomous(name = "RightInspiredAwardAutonomous", group = "Autonomous")
public class RightSideAutonomous extends LinearOpMode {

    public class ArmSlidesClaw {
        private DcMotorEx arm, leftSlide, rightSlide;
        private Servo leftClaw, rightClaw, claw;

        private ElapsedTime timer = new ElapsedTime();

        public ArmSlidesClaw(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
            rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

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

        public void moveSlidesVel(int targetSlides, double power) {
            leftSlide.setTargetPosition(targetSlides);
            rightSlide.setTargetPosition(targetSlides);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlide.setVelocity(power);
            rightSlide.setVelocity(power);
        }

        public void wristUp() {
            leftClaw.setPosition(-1);
            rightClaw.setPosition(0.666);
        }

        public void wristDown() {
            leftClaw.setPosition(0.466);
            rightClaw.setPosition(0.35);
        }

        public void wristPlaceSpecimen() {
            leftClaw.setPosition(0.088);
            rightClaw.setPosition(0.652);
        }

        public void wristPickUp() {
            leftClaw.setPosition(0.182);
            rightClaw.setPosition(0.522);
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

            private int slidesPlace = 900;

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
                    wristPlaceSpecimen();

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

                if (timer.seconds() <= 3.7) {
                    if (timer.seconds() > 1.8) {
                        moveArm(armPlacePosition, 1);
                    }
                    closeClaw();
                    wristPlaceSpecimen();

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
            return new RightSideAutonomous.ArmSlidesClaw.PlaceSecondSpecimen();
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
                    wristPlaceSpecimen();

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

        public class MoveSlidesDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveSlidesVel(0, 5500);
                return true;
            }
        }

        public Action moveSlidesDown() {
            return new RightSideAutonomous.ArmSlidesClaw.MoveSlidesDown();
        }

    }





    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(6, -62, Math.toRadians(270));
        RightMecanumDrive drive = new RightMecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        TrajectoryActionBuilder placeFirstSpecimen = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(109))
                .lineToYLinearHeading(-33, Math.toRadians(270))
                ;

        TrajectoryActionBuilder pushPath = placeFirstSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(20, -40), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(47, -12), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(47, -52), new TranslationalVelConstraint(120))
                .strafeToLinearHeading(new Vector2d(42, -12), Math.toRadians(270.0), new TranslationalVelConstraint(80))
                ;


        TrajectoryActionBuilder pickUpSecondSpecimen = pushPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(54, -13))
                .strafeToConstantHeading(new Vector2d(54, -64.5))
                ;

        TrajectoryActionBuilder placeSecondSpecimen = pickUpSecondSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-2, -33), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder pickUpThirdSpecimen = placeSecondSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35.80, -60), Math.toRadians(270.00))
                ;

        TrajectoryActionBuilder placeThirdSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(1, -32), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder pickUpForthSpecimen = placeThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35.80, -60), Math.toRadians(270.00))
                ;

        TrajectoryActionBuilder placeForthSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(3.4, -32), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder park = placeThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(45.09, -60.00, Math.toRadians(320.00)), Math.toRadians(320.00), new TranslationalVelConstraint(200));
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

                        armslidesclaw.moveSlidesDown()

                )
        );

    }
}