package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
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
@Autonomous(name = "LeftSampleAuto", group = "Autonomous")
public class LeftSampleAuto extends LinearOpMode {
    Wrist wrist = new Wrist(this);

    private OpenCvCamera webcam;

    public class ArmSlidesClaw {
        private DcMotor arm, leftSlide, rightSlide;

        public DcMotor frontLeft, frontRight, backLeft, backRight;

        private Servo claw;

        private ElapsedTime timer = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        private double xAlign = 0;
        private double RealYValue = 0.0;
        private double RealAngleValue = 0.0;

        private double clawClose = 0.8;
        private double clawOpen = 0;

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
        public class PlaceSample implements Action {
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
                if (armReachedTarget(1600, 1000)) {
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
                    wrist.PickUp0();
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
                    moveSlides(400, 1);
                }

                if (slidesReachedTarget(400, 900) && !slidesMoveDownAfterSample) {
                    timer.reset();
                    resetArm();
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
                    moveSlides(400, 1);
                }

                if (slidesReachedTarget(400, 900) && !slidesMoveDownAfterSample) {
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
                    wrist.PickUp0();
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

        TrajectoryActionBuilder placeSample1 = drive.actionBuilder(new Pose2d(-37, -62, Math.toRadians(0)))
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(180));

        TrajectoryActionBuilder grabSample2 = placeSample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-47, -41), Math.toRadians(90));

        TrajectoryActionBuilder placeSample2 = grabSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45));

        TrajectoryActionBuilder grabSample3 = placeSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-57, -41), Math.toRadians(90));

        TrajectoryActionBuilder placeSample3 = grabSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45));

        TrajectoryActionBuilder grabSample4 = placeSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-57, -43), Math.toRadians(115));

        TrajectoryActionBuilder placeSample4 = grabSample4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45));

        TrajectoryActionBuilder grabSample5 = placeSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-20, -8, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder grabSample6 = placeSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25, -5, Math.toRadians(0)), Math.toRadians(0));

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

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
                        ),
                        new ParallelAction(
                                grabSample5.build(),
                                new ParallelAction(
                                        armslidesclaw.resetArm(),
                                        armslidesclaw.resetSlides()
                                )
                        )
                )
        );

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

        /*
        Actions.runBlocking(
                new SequentialAction(
                        armslidesclaw.placeSample()
                )
        );
         */
    }
}