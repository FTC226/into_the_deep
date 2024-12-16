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
        }

        public void moveArm(int targetArm, double power) {
            arm.setTargetPosition(targetArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);
        }

        public void moveSlides(int targetSlides, double power) {
            leftSlide.setTargetPosition(-targetSlides);
            rightSlide.setTargetPosition(targetSlides);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlide.setPower(power);
            rightSlide.setPower(power);
        }

        public void waitSeconds(int seconds) {
            timer.reset();

            while (timer.seconds() < seconds) {

            }
        }

        public void moveWrist(boolean moveUp, double seconds) {

        }

        public class PlaceSpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveArm(-1270, 0.8);

                if (Math.abs(arm.getCurrentPosition() + 1270) < 40) {
                    moveSlides(790, 0.8);
                }

                if (Math.abs(arm.getCurrentPosition() + 1270) < 40 && (Math.abs(leftSlide.getCurrentPosition() + 790) > 20) && (Math.abs(rightSlide.getCurrentPosition() - 790) > 20)) {
                    return false;
                }

                return true;
            }
        }
        public Action placeSpecimen() {
            return new PlaceSpecimen();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(8, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        TrajectoryActionBuilder placeFirstSpecimen = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-3.00, -31.50, Math.toRadians(270.00)), Math.toRadians(90.00))
                ;

        TrajectoryActionBuilder pushPath = placeFirstSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(24, -38.5), Math.toRadians(0.00))
                .splineToLinearHeading(new Pose2d(45, -15, Math.toRadians(0.00)), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(45, -52))
                .strafeToConstantHeading(new Vector2d(45, -15))
                .splineToLinearHeading(new Pose2d(55, -15, Math.toRadians(0.00)), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(55, -52))
                .splineToLinearHeading(new Pose2d(30.00, -60.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(0.001)
                .splineToLinearHeading(new Pose2d(45.00, -60.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                ;
        TrajectoryActionBuilder placeSecondSpecimen = pushPath.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-1.00, -31.50, Math.toRadians(270.00)), Math.toRadians(90.00))

                ;
        TrajectoryActionBuilder pickUpThirdSpecimen = placeSecondSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(30.00, -60.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(0.001)
                .splineToLinearHeading(new Pose2d(45.00, -60.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                ;
        TrajectoryActionBuilder placeThirdSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(1.00, -31.50, Math.toRadians(270.00)), Math.toRadians(90.00))

                ;

        TrajectoryActionBuilder pickUpForthSpecimen = placeThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(30.00, -60.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(0.001)
                .splineToLinearHeading(new Pose2d(45.00, -60.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                ;

        TrajectoryActionBuilder placeForthSpecimen = pickUpThirdSpecimen.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(2.50, -31.50, Math.toRadians(270.00)), Math.toRadians(90.00))

                ;

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        placeFirstSpecimen.build(),
                        pushPath.build(),
                        placeSecondSpecimen.build(),
                        pickUpThirdSpecimen.build(),
                        placeThirdSpecimen.build(),
                        pickUpForthSpecimen.build(),
                        placeForthSpecimen.build()
                )
        );

    }
}