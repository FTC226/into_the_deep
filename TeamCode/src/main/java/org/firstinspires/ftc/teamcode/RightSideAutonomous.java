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

        public void closeClaw() {
            claw.setPosition(1);
        }public void openClaw() {
            claw.setPosition(-1);
        }

        public void wristDown() {
//            leftClaw.setPosition(0.358);
//            rightClaw.setPosition(0.684);
            leftClaw.setPosition(0.618);
            rightClaw.setPosition(0.358);
        }

        public void wristUp() {
            leftClaw.setPosition(0.226);
            rightClaw.setPosition(0.798);
        }

        public void wristPlaceSpecimen() {
            leftClaw.setPosition(0.816);
            rightClaw.setPosition(0.578);
        }

        public void wristPickUp() {
            leftClaw.setPosition(0.85);
            rightClaw.setPosition(0.514);
        }

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(leftSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
        }

        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }

        public class PlaceFirstSpecimen implements Action {
            private int armPlacePosition = 1570;

            private int slidesReady = 700;
            private int slidesPlace = 900;

            private boolean isRest = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isRest) {
                    timer.seconds();
                    isRest = true;
                }

                if (timer.seconds() <= 2.6) {
                    moveArm(armPlacePosition, 1);
                    closeClaw();
                    wristPlaceSpecimen();

                    if (armReachedTarget(armPlacePosition, 300)) {
                        moveSlides(slidesReady, 1);
                    }

                    if (slidesReachedTarget(slidesReady, 30)) {
                        moveSlides(slidesPlace, 1);
                    }
                }

                if (timer.seconds() > 2.7) {
                    openClaw();
                    moveSlides(0, 1);
                    moveArm(0, 1);
                }



                if (timer.seconds() > 2.9) {
                    timer.reset();
                    isRest = false;
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



        public class PlaceOtherSpecimen implements Action {
            private int armPlacePosition = 1570;

            private int slidesReady = 700;
            private int slidesPlace = 900;

            private boolean isReset = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    resetTimer();
                    isReset = true;
                }

                if (timer.seconds() <= 2.8) {
                    moveArm(armPlacePosition, 1);
                    closeClaw();
                    wristPlaceSpecimen();

                    if (armReachedTarget(armPlacePosition, 300)) {
                        moveSlides(slidesReady, 1);
                    }

                    if (slidesReachedTarget(slidesReady, 30)) {
                        moveSlides(slidesPlace, 1);
                    }
                }

                if (timer.seconds() > 2.9) {
                    openClaw();
                    moveSlides(0, 1);
                    moveArm(0, 1);
                }



                if (timer.seconds() > 3.1) {
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
            private int armPickUpPosition = 560;

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

    }





    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(8, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        TrajectoryActionBuilder placeFirstSpecimen = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(105))
                .lineToYLinearHeading(-32, Math.toRadians(270))
                ;

        TrajectoryActionBuilder pushPath = placeFirstSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(30, -40), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(42, -14), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(42, -52), new TranslationalVelConstraint(120))
                .strafeToLinearHeading(new Vector2d(42, -15), Math.toRadians(270.0), new TranslationalVelConstraint(120))

                ;


        TrajectoryActionBuilder pickUpSecondSpecimen = pushPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(53, -15))
                .strafeToConstantHeading(new Vector2d(53, -61), new TranslationalVelConstraint(120))
                ;


        TrajectoryActionBuilder placeSecondSpecimen = pickUpSecondSpecimen.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(8, -58), new TranslationalVelConstraint(120))
                .strafeToConstantHeading(new Vector2d(-1, -32), new TranslationalVelConstraint(120))
                ;



        TrajectoryActionBuilder pickUpThirdSpecimen = placeSecondSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -52), Math.toRadians(270.00))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(35, -61.00))
                ;

        TrajectoryActionBuilder placeThirdSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(8, -58), new TranslationalVelConstraint(120))
                .strafeToConstantHeading(new Vector2d(-1, -32), new TranslationalVelConstraint(120))
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

//        armslidesclaw.resetTimer();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                placeFirstSpecimen.build(),
                                armslidesclaw.placeFirstSpecimen()
                        ),

                        pushPath.build(),
//
                        new ParallelAction(
                                pickUpSecondSpecimen.build(),
                                armslidesclaw.pickUpSecondSpecimen()
                        ),
//
                        new ParallelAction(
                                placeSecondSpecimen.build(),
                                armslidesclaw.placeOtherSpecimen()
                        )

//                        new ParallelAction(
//                                armslidesclaw.pickUpSecondSpecimen()

//                        ),
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