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
@Autonomous(name = "LeftSideAutonomous", group = "Autonomous")
public class LeftSideAutonomous extends LinearOpMode {
    Wrist wrist = new Wrist(this);
    Camera camera = new Camera(this);

    private OpenCvCamera webcam;

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
            camera.init();

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

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

            webcam.setPipeline(camera);

            camera.setColor("blue");

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

                    dashboard.startCameraStream(webcam, 60);
                    telemetry.addData("Status", "Camera started");
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Error", "Camera failed to open with error code: " + errorCode);
                    telemetry.update();
                }
            });

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

         CAMERA

        */

        public void search() {
            double moveX = camera.realX();
            double moveY = 0;
            double realAngle = camera.realAngle();
            boolean centralized = false;

            //move the claw first
            if (realAngle != -90) {
                wrist.setRotateServo(0.00377778*(realAngle)+0.16);
            } else {
                wrist.setRotateServo(0.00377778*(Math.abs(realAngle))+0.16);
            }

            double magnitude = Math.sqrt(Math.pow(moveX,2)+Math.pow(moveY,2));
            double deltaX = moveX/magnitude;
            double deltaY = moveY/magnitude;
            double denominator = Math.max(Math.abs(moveX), 1);
            double searchBig = 0.4;
            double searchSmall = 0.3;



            if (Math.abs(moveX)<=1.5) {
                frontLeft.setPower(((moveX) / denominator) * searchSmall);
                frontRight.setPower(((-moveX) / denominator) * searchSmall);
                backLeft.setPower(((-moveX) / denominator) * searchSmall);
                backRight.setPower(((moveX) / denominator) * searchSmall);
            } else {
                frontLeft.setPower(((moveX) / denominator) * searchBig);
                frontRight.setPower(((-moveX) / denominator) * searchBig);
                backLeft.setPower(((-moveX) / denominator) * searchBig);
                backRight.setPower(((moveX) / denominator) * searchBig);
            }
        }

        public double realXtoMM(double x){
            double inches = 2.3027*x;
            return 0.89887640449*inches;
        }


        public double realXtoMM(){
            double inches = camera.getRealXMatrix();
            RealYValue = camera.realY();
            RealAngleValue = camera.realAngle();

            return inches;
        }

        public double angleOrientation(){
            if (RealYValue != -90) {
                return (0.00377778*(RealAngleValue)+0.16);
            } else {
                return (0.00377778*(Math.abs(RealAngleValue))+0.16);
            }
        }

        public int extendedSearchVal(){
            double moveX = 0;
            double moveY = RealYValue;


            return (int)(110.263*camera.getRealYMatrix());
        }

        public class AlignRobot implements  Action {
            double xValue = camera.realX();
            boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    timer.reset();
                    isReset = true;
                }
                wrist.middleWrist();
                search();
                telemetry.addData("Real X:", camera.realX());
                telemetry.addData("Real Y:", camera.realY());
                telemetry.addData("calcVal", extendedSearchVal());
                telemetry.addData("Real Angle:", camera.realAngle());
                return !(timer.seconds() > 4);

//                return !(Math.abs(xValue) < 0.2);
            }
        }
        public Action alignRobot() {
            return new AlignRobot();
        }

        public class NewRobotAlign implements Action {
            double xValue = realXtoMM();
            boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset) {
                    claw.setPosition(clawOpen);
                    timer.reset();
                    isReset = true;
                }
                xAlign = xValue;


                telemetry.addData("xAlign", xAlign);
                telemetry.addData("xValue ", xValue);
                telemetry.addData("realX ", camera.realX());

                telemetry.update();


                return !(timer.seconds() > 4);
            }
        }
        public Action newRobotAlign() { return new NewRobotAlign();}

        public class GrabSampleSubmersible implements Action {
            private boolean closeClaw = false;
            boolean isExtended = false;
            int slidesTargetPosition = extendedSearchVal();
            boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                wrist.setRotateServo(angleOrientation());
                if (!isReset) {
                    timer.reset();
                    isReset = true;
                }

                if (!isExtended && timer.seconds() > 0.5) {
                    claw.setPosition(clawOpen);
                    moveSlides(extendedSearchVal(), 1);
                    slidesTargetPosition = extendedSearchVal();
                    isExtended = true;
                }


                if (slidesReachedTarget(slidesTargetPosition, slidesTargetPosition/2) && !closeClaw) {
                    wrist.moveWristDown();
                }


                if (slidesReachedTarget(slidesTargetPosition, 50) && !closeClaw) {
                    closeClaw = true;
                    claw.setPosition(clawClose);
                    timer.reset();
                }

                telemetry.addData("Slide Left", leftSlide.getCurrentPosition());
                telemetry.addData("Slide Right", rightSlide.getCurrentPosition());

                return !closeClaw || !(timer.seconds() > 1);
            }
        }
        public Action grabSampleSubmersible() { return new GrabSampleSubmersible();}

        public class StopRobot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setRotateServo(wrist.rotateServo.getPosition());
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                return false;
            }
        }
        public Action stopRobot() { return new StopRobot(); }

        public class ResetAfterSubmersible implements Action {
            private boolean slidesRetract = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!slidesRetract) {
                    moveSlides(400, 1);
                    wrist.PlaceSample();
                    slidesRetract = true;
                }

                if (slidesReachedTarget(400, 30) && slidesRetract) {
                    return false;
                }

                return true;
            }
        }
        public Action resetAfterSubmersible() { return new ResetAfterSubmersible();}

        public class ResetWristAfterSubmersible implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                wrist.PlaceSample();

                return false;
            }
        }
        public Action resetWristAfterSubmersible() { return new ResetWristAfterSubmersible();}

        public class ResetWristAfterPlaceSample implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.middleWrist();
                claw.setPosition(clawOpen);

                return false;
            }
        }
        public Action resetWristAfterPlaceSample() { return new ResetWristAfterPlaceSample();}


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
                if (armReachedTarget(1600, 800)) {
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

        double sampleCoord = -50.2;

        TrajectoryActionBuilder placeSample1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(sampleCoord, sampleCoord, Math.toRadians(45)), Math.toRadians(180))
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample2 = placeSample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-46, -41), Math.toRadians(89));

        TrajectoryActionBuilder placeSample2 = grabSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(sampleCoord, sampleCoord), Math.toRadians(45));

        TrajectoryActionBuilder grabSample3 = placeSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-57, -41), Math.toRadians(85));

        TrajectoryActionBuilder placeSample3 = grabSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(sampleCoord, sampleCoord), Math.toRadians(45));

        TrajectoryActionBuilder grabSample4 = placeSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-57, -43), Math.toRadians(110));

        TrajectoryActionBuilder placeSample4 = grabSample4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(sampleCoord, sampleCoord), Math.toRadians(45));

        TrajectoryActionBuilder grabSample5 = placeSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-20, -5, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder grabSample6 = placeSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-20, 0, Math.toRadians(0)), Math.toRadians(0));

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        telemetry.addData("real_area: ", camera.getRealArea());
        telemetry.update();

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
                                armslidesclaw.resetWristAfterPlaceSample(),
                                new SequentialAction(
                                        armslidesclaw.resetSlides(),
                                        armslidesclaw.resetArm()
                                )
                        )
                )
        );

        Vector2d grabSample5Pose = new Vector2d(-20, -5+armslidesclaw.realXtoMM());

        TrajectoryActionBuilder alignRobot = grabSample5.endTrajectory().fresh()
                .strafeToConstantHeading(grabSample5Pose);

        Actions.runBlocking(
                new SequentialAction(
                        alignRobot.build(),
                        armslidesclaw.stopRobot(),
                        armslidesclaw.grabSampleSubmersible()
                )
        );

        TrajectoryActionBuilder placeSample5 = alignRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(sampleCoord, sampleCoord, Math.toRadians(45)), Math.toRadians(270));

        Actions.runBlocking(
                new SequentialAction(
                        armslidesclaw.resetWristAfterSubmersible(),
                        new ParallelAction(
                                placeSample5.build(),
                                new SequentialAction(
                                        armslidesclaw.resetAfterSubmersible(),
                                        armslidesclaw.placeSampleSubmersible()
                                )
                        ),
                        new ParallelAction(
                                grabSample6.build(),
                                armslidesclaw.resetWristAfterPlaceSample(),
                                new SequentialAction(
                                        armslidesclaw.resetSlides(),
                                        armslidesclaw.resetArm()
                                )
                        )
                )
        );

        Vector2d grabSample6Pose = new Vector2d(-20, 0+armslidesclaw.realXtoMM()); // + 9

        TrajectoryActionBuilder alignRobot1 = grabSample6.endTrajectory().fresh()
                .strafeToConstantHeading(grabSample6Pose);

        Actions.runBlocking(
                new SequentialAction(
                        alignRobot1.build(),
                        armslidesclaw.stopRobot(),
                        armslidesclaw.grabSampleSubmersible()
                )
        );

        TrajectoryActionBuilder placeSample6 = alignRobot1.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(sampleCoord, sampleCoord, Math.toRadians(45)), Math.toRadians(270));

        Actions.runBlocking(
                new SequentialAction(
                        armslidesclaw.resetWristAfterSubmersible(),
                        new ParallelAction(
                                placeSample6.build(),
                                new SequentialAction(
                                        armslidesclaw.resetAfterSubmersible(),
                                        armslidesclaw.placeSampleSubmersible()
                                )
                        )
                )
        );
    }

}