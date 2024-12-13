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

        TrajectoryActionBuilder placeSpecimenPath = drive.actionBuilder(initialPose)
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
                .lineToXLinearHeading(55,Math.toRadians(0))
                .setTangent(Math.toRadians(270))
                .lineToYLinearHeading(-52, Math.toRadians(0));


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

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        placeSpecimenPath.build(),
                        firstSamplePath.build(),
                        pushFirstSample.build(),
                        goBackToFirstSample.build(),
                        secondSamplePath.build(),
                        pickUpSpecimenSecond.build(),
                        placeSpecimenSecond.build()
//                        pickUpSpecimenThird.build(),
//                        placeSpecimenThird.build(),
//                        pickUpSpecimenFourth.build(),
//                        placeSpecimenFourth.build()
                )
        );

    }
}