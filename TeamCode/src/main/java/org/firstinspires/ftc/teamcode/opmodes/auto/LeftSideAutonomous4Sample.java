package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subss.Camera;
import org.firstinspires.ftc.teamcode.subss.Wrist;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "LeftSideAutonomous4Sample", group = "Autonomous")
public class    LeftSideAutonomous4Sample extends LinearOpMode {
    Wrist wrist = new Wrist(this);

    public class ArmSlidesClaw {
        double RealYValue = 0.0;
        double RealAngleValue = 0.0;

        public DcMotor frontLeft, frontRight, backLeft, backRight;

        private DcMotor arm, leftSlide, rightSlide;

        private Servo claw;

        private double clawClose = 0.8;
        private double clawOpen = 0;

        private ElapsedTime timer = new ElapsedTime();

        private double xAlign = 0;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        public ArmSlidesClaw(HardwareMap hardwareMap) {
            wrist.init();

            arm = hardwareMap.get(DcMotor.class, "arm");
            leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
            rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

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

            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            moveArm(800, 1);
            claw.setPosition(clawClose);
        }

        /*

        HELPER METHODS

         */

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

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(leftSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
        }

        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }

        /*

        AUTO ACTIONS

         */

        public class PlaceSampleSubmersible implements Action {
            private boolean wristPlaceSample = false;
            private boolean resetWrist = false;
            private boolean wristSample = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Move arm
                moveArm(1600, 1);

                if (!wristSample) {
                    wrist.PlaceSample();
                    claw.setPosition(clawClose);
                }

                // Once arm reached target, move slides
                if (armReachedTarget(1600, 100)) {
                    moveSlides(2100, 1);
                    wristSample = true;
                }

                // Once slide reached target, move wrist
                if (slidesReachedTarget(2100, 20) && !wristPlaceSample) {
                    timer.reset();
                    claw.setPosition(clawOpen);
                    wristPlaceSample = true;
                }

                // Once claw is opened and timer reached target, reset wrist
                if (wristPlaceSample && timer.seconds() > 0.2 && !resetWrist) {
                    wrist.PickUp0Auto();
                    return false;
                }

                return true;
            }
        }
        public Action placeSampleSubmersible() {
            return new PlaceSampleSubmersible();
        }

        public class PlaceSample1 implements Action {
            private boolean moveArmMedium = false;
            private boolean wristPlaceSample = false;
            private boolean resetWrist = false;
            private boolean wristSample = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Move arm
                moveArm(1650, 1);

                if (!wristSample) {
                    wrist.PlaceSample();
                    claw.setPosition(clawClose);
                }

                // Once arm reached target, move slides
                if (armReachedTarget(1650, 400)) {
                    moveSlides(2150, 1);
                    wristSample = true;
                }

                // Once slide reached target, move wrist
                if (slidesReachedTarget(2150, 20) && !wristPlaceSample) {
                    timer.reset();
                    claw.setPosition(clawOpen);
                    wristPlaceSample = true;
                }

                // Once claw is opened and timer reached target, reset wrist
                if (wristPlaceSample && timer.seconds() > 0.4 && !resetWrist) {
                    return false;
                }

                return true;
            }
        }
        public Action placeSample1() {
            return new PlaceSample1();
        }


        public class PlaceSample implements Action {
            private boolean moveArmMedium = false;
            private boolean wristPlaceSample = false;
            private boolean resetWrist = false;
            private boolean wristSample = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Move arm
                if (!moveArmMedium) {
                    moveArm(1300, 1);
                    moveArmMedium = true;
                }

                if (!wristSample) {
                    wrist.PlaceSample();
                    claw.setPosition(clawClose);
                }

                // Once arm reached target, move slides
                if (armReachedTarget(1300, 200)) {
                    moveSlides(2150, 1);
                    moveArm(1650, 1);
                    wristSample = true;
                }

                // Once slide reached target, move wrist
                if (slidesReachedTarget(2150, 20) && !wristPlaceSample) {
                    timer.reset();
                    claw.setPosition(clawOpen);
                    wristPlaceSample = true;
                }

                // Once claw is opened and timer reached target, reset wrist
                if (wristPlaceSample && timer.seconds() > 0.4 && !resetWrist) {
                    return false;
                }

                return true;
            }
        }
        public Action placeSample() {
            return new PlaceSample();
        }

        public class GrabSample implements Action {
            private boolean slidesMoveDownAfterSample = false;
            private boolean resetArm = false;
            private boolean extendSlidesSample = false;
            private boolean closeClaw = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!slidesMoveDownAfterSample) {
                    moveSlides(200, 1);
                }

                if (slidesReachedTarget(200, 800) && !slidesMoveDownAfterSample) {
                    timer.reset();
                    resetArm();
                    wrist.PickUp0Auto();
                    slidesMoveDownAfterSample = true;
                }

                if (!resetArm && slidesMoveDownAfterSample) {
                    arm.setTargetPosition(0);
                    resetArm = true;
                    timer.reset();

                    arm.setPower(-1);
                }

                if (resetArm && timer.seconds() > 0.6) {
                    moveSlides(780, 1);
                    extendSlidesSample = true;
                }

                if (resetArm && timer.seconds() > 1) {
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                if (slidesReachedTarget(780, 15) && !closeClaw && extendSlidesSample) {
                    timer.reset();
                    claw.setPosition(clawClose);
                    closeClaw = true;
                }

                if (closeClaw && timer.seconds() > 0.8) {
                    moveSlides(400, 1);
                    return false;
                }

                return true;
            }
        }
        public Action grabSample() { return new GrabSample();}

        public class GrabSample4 implements Action {
            private boolean slidesMoveDownAfterSample = false;
            private boolean resetArm = false;
            private boolean extendSlidesSample = false;
            private boolean closeClaw = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!slidesMoveDownAfterSample) {
                    moveSlides(200, 1);
                }

                if (slidesReachedTarget(200, 800) && !slidesMoveDownAfterSample) {
                    timer.reset();
                    resetArm();
                    slidesMoveDownAfterSample = true;
                }

                if (!resetArm && slidesMoveDownAfterSample) {
                    arm.setTargetPosition(0);
                    resetArm = true;
                    wrist.PickUp45Right();
                    timer.reset();

                    arm.setPower(-1);
                }

                if (resetArm && timer.seconds() > 0.6) {
                    moveSlides(1130, 1);
                    extendSlidesSample = true;
                }

                if (resetArm && timer.seconds() > 1) {
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                if (slidesReachedTarget(1130, 15) && !closeClaw && extendSlidesSample) {
                    timer.reset();
                    claw.setPosition(clawClose);
                    closeClaw = true;
                }

                if (closeClaw && timer.seconds() > 0.8) {
                    moveSlides(400, 1);
                    wrist.PickUp0Auto();
                    return false;
                }

                return true;
            }
        }
        public Action grabSample4() { return new GrabSample4();}

        public class ResetArm implements Action {
            private boolean resetArm = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!resetArm) {
                    arm.setTargetPosition(0);
                    resetArm = true;
                    timer.reset();

                    arm.setPower(-1);
                }

                if (resetArm && timer.seconds() > 0.8) {
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                }

                return true;
            }
        }
        public Action resetArm() { return new ResetArm(); }

        public class ResetSlides implements Action {
            private boolean resetSlides = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!resetSlides) {
                    leftSlide.setTargetPosition(0);
                    rightSlide.setTargetPosition(0);
                    resetSlides = true;
                    timer.reset();

                    leftSlide.setPower(-1);
                    rightSlide.setPower(-1);
                }

                if (resetSlides && timer.seconds() > 1.2) {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                }

                return true;
            }
        }
        public Action resetSlides() { return new ResetSlides(); }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-37, -62, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder placeSample1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-50, -48), Math.toRadians(45));

        TrajectoryActionBuilder grabSample2 = placeSample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-46, -41), Math.toRadians(88));

        TrajectoryActionBuilder placeSample2 = grabSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50.4, -48.4), Math.toRadians(45));

        TrajectoryActionBuilder grabSample3 = placeSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-57, -41), Math.toRadians(80));

        TrajectoryActionBuilder placeSample3 = grabSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50.4, -48.4), Math.toRadians(45));

        TrajectoryActionBuilder grabSample4 = placeSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-57, -43), Math.toRadians(118));

        TrajectoryActionBuilder placeSample4 = grabSample4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50.4, -48.4), Math.toRadians(45));

        TrajectoryActionBuilder grabSample5 = placeSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-20, -5, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder grabSample6 = placeSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-20, 0, Math.toRadians(0)), Math.toRadians(0));

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        /*
        Actions.runBlocking(
                new SequentialAction(
                        placeSample1.build(),
                        grabSample2.build(),
                        placeSample2.build(),
                        grabSample3.build(),
                        placeSample3.build(),
                        grabSample4.build(),
                        placeSample4.build()
                )
        );
         */

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                placeSample1.build(),
                                armslidesclaw.placeSample()
                        ),
                        new ParallelAction(
                                grabSample2.build(),
                                armslidesclaw.grabSample()
                        ),
                        new ParallelAction(
                                placeSample2.build(),
                                armslidesclaw.placeSample()
                        ),
                        new ParallelAction(
                                grabSample3.build(),
                                armslidesclaw.grabSample()
                        ),
                        new ParallelAction(
                                placeSample3.build(),
                                armslidesclaw.placeSample()
                        ),
                        new ParallelAction(
                                grabSample4.build(),
                                armslidesclaw.grabSample4()
                        ),
                        new ParallelAction(
                                placeSample4.build(),
                                armslidesclaw.placeSample()
                        )
                )
        );
    }

}