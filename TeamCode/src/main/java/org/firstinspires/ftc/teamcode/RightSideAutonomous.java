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
            leftClaw.setPosition(0.256);
            rightClaw.setPosition(0.636);
        }

        public void wristUp() {
            leftClaw.setPosition(0.612);
            rightClaw.setPosition(0.266);
        }

        public void wristPlaceSample() {
            leftClaw.setPosition(0.508);
            rightClaw.setPosition(0.398);
        }

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(leftSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
        }

        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }

        public class PlaceSpecimen implements Action {
            private int targetArm = 1700;
            private int targetSlides = 900;

            private boolean armMoveDown = false;
            private boolean slideTimerStart = false;
            private boolean clawTimerStart = false;
            private boolean waitClawRelease = false;
            private boolean waitWristDown = false;
            private boolean slidesMoveDown = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!armMoveDown) {
                    moveArm(targetArm, 1);
                }

                if (armReachedTarget(targetArm, 10) && !slideTimerStart) {
                    slideTimerStart = true;
                    timer.reset();
                }

                if (slideTimerStart && timer.seconds() > 1.5) {
                    moveSlides(targetSlides, 0.6);
                }

                if (slidesReachedTarget(targetSlides, 10) && !clawTimerStart) {
                    clawTimerStart = true;
                    timer.reset();

                    wristPlaceSample();
                }

                if (clawTimerStart && timer.seconds() > 1 && !waitClawRelease) {
                    claw.setPosition(0);
                    waitClawRelease = true;
                    timer.reset();
                }

                if (waitClawRelease && timer.seconds() > 0.5 && !waitWristDown) {
                    wristDown();
                    waitWristDown = true;
                    timer.reset();
                }

                if (waitWristDown && timer.seconds() > 1 && !slidesMoveDown) {
                    return false;
                }

                telemetry.addData("Arm ", arm.getCurrentPosition());
                telemetry.addData("Arm target ", arm.getTargetPosition());
                telemetry.addData("Left slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right slide ", rightSlide.getCurrentPosition());
                telemetry.addData("Left slide target ", leftSlide.getTargetPosition());
                telemetry.addData("Right slide target ", rightSlide.getTargetPosition());
                telemetry.update();

                return true;
            }
        }
        public Action placeSpecimen() {
            return new RightSideAutonomous.ArmSlidesClaw.PlaceSpecimen();
        }

        public class GrabSample implements Action {
            private int targetSlides = 1100;
            private boolean clawTimerStarted = false;
            private boolean retractSlides = false;
            private boolean slidesExtended = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!slidesExtended) {
                    moveSlides(targetSlides, 1);
                    claw.setPosition(0);
                }

                wristDown();

                if (slidesReachedTarget(targetSlides, 10) && !clawTimerStarted) {
                    clawTimerStarted = true;
                    retractSlides = true;
                    slidesExtended = true;
                    claw.setPosition(1);
                    timer.reset();
                }

                if (clawTimerStarted && timer.seconds() > 0.5) {
                    moveSlides(0, 1);
                }

                return !retractSlides || !slidesReachedTarget(0, 10);
            }
        }
        public Action grabSample() {
            return new RightSideAutonomous.ArmSlidesClaw.GrabSample();
        }

        public class ResetEncoders implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                return false;
            }
        }
        public Action resetEncoders() {
            return new RightSideAutonomous.ArmSlidesClaw.ResetEncoders();
        }

        public class ResetArmSlides implements Action {
            private boolean resetTimer = false;
            private boolean runSlides = false;
            private boolean slidesDown = false;
            private boolean runArm = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setTargetPosition(0);
                leftSlide.setTargetPosition(0);
                rightSlide.setTargetPosition(0);

                if (!resetTimer) {
                    timer.reset();
                    resetTimer = true;
                    runSlides = true;
                }

                if (runSlides && timer.seconds() < 2) {
                    leftSlide.setPower(-1);
                    rightSlide.setPower(-1);
                }

                if (timer.seconds() > 2 && runSlides) {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    runSlides = false;
                    slidesDown = true;
                }

                if (slidesDown && !runArm) {
                    timer.reset();
                    runArm = true;
                }

                if (runArm && timer.seconds() < 2) {
                    arm.setPower(-1);
                }

                telemetry.addData("Arm current ", arm.getCurrentPosition());
                telemetry.addData("Arm target ", arm.getTargetPosition());
                telemetry.addData("Left slide current ", leftSlide.getCurrentPosition());
                telemetry.addData("left slide target ", leftSlide.getTargetPosition());
                telemetry.update();

                return !(timer.seconds() > 2) || !runArm;
            }
        }
        public Action resetArmSlides() {
            return new RightSideAutonomous.ArmSlidesClaw.ResetArmSlides();
        }

        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);

                return false;
            }
        }
        public Action closeClaw() {
            return new RightSideAutonomous.ArmSlidesClaw.CloseClaw();
        }

        public class OpenClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);

                return false;
            }
        }
        public Action openClaw() {
            return new RightSideAutonomous.ArmSlidesClaw.OpenClaw();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(8, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        TrajectoryActionBuilder placeFirstSpecimen = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-3.00, -34, Math.toRadians(270.00)), Math.toRadians(90.00))
                .waitSeconds(1)
                ;

        TrajectoryActionBuilder pushPath = placeFirstSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(24, -38.5), Math.toRadians(0.00))
                .splineToLinearHeading(new Pose2d(45, -15, Math.toRadians(0.00)), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(45, -52))
                .strafeToConstantHeading(new Vector2d(45, -13))
                .splineToLinearHeading(new Pose2d(58, -13, Math.toRadians(0.00)), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(58, -52))
//                .splineToLinearHeading(new Pose2d(30.00, -58.00, Math.toRadians(0.00)), Math.toRadians(0.00))

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
                )
        );

    }
}