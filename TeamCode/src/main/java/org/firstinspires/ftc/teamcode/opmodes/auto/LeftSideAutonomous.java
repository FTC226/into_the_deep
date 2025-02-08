package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subss.Camera;
import org.firstinspires.ftc.teamcode.subss.Slides;
import org.firstinspires.ftc.teamcode.subss.Wrist;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Stack;

@Config
@Autonomous(name = "LeftSideAutonomous", group = "Autonomous")
public class LeftSideAutonomous extends LinearOpMode {
    Wrist wrist = new Wrist(this);
    Camera camera = new Camera(this);
    Slides slides = new Slides(this);

    private OpenCvCamera webcam;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    public class ArmSlidesClaw {
        public int count0 = 0;
        public int count45l = 0;
        public int count45r = 0;
        public int count90 = 0;


        private DcMotor arm, leftSlide, rightSlide;

        public DcMotor frontLeft, frontRight, backLeft, backRight;


        private Servo claw;

        private ElapsedTime timer = new ElapsedTime();

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

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

            // Initialize the pipeline

            webcam.setPipeline(camera);

            camera.setColor("blue");



            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    webcam.startStreaming(640, 360, OpenCvCameraRotation.UPSIDE_DOWN);

                    dashboard.startCameraStream(webcam, 60);
                    telemetry.addData("Status", "Camera started");


                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Error", "Camera failed to open with error code: " + errorCode);
                    telemetry.update();
                }
            });
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

        public void search() {

            double moveX = camera.realX();
            double moveY = 0;
            double realAngle = camera.realAngle();
//            boolean centralized = false;

            //move the claw first
            if (realAngle > -90 && realAngle < -20) {
                count45l++;
            }
            else if (realAngle <= 0 || realAngle > 50) {
                wrist.SearchPickUp90();
                count90++;
            }
            else if (realAngle > 20 && realAngle < 50) { //20<realAngle && realAngle<50
                count45r++;

            }
            if (realAngle < 20 && realAngle > -20) {
                wrist.SearchPickUp0();
                count0++;

            }

//        if (realAngle )



            double magnitude = Math.sqrt(Math.pow(moveX,2)+Math.pow(moveY,2));
            double deltaX = moveX/magnitude;
            double deltaY = moveY/magnitude;
            double denominator = Math.max(Math.abs(moveX), 1);
            double searchBig = 0.4;
            double searchSmall = 0.6;



            if (Math.abs(moveX)<=0.5) {
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

        public int extendedSearchVal(){
            double moveX = 0;
            double moveY = camera.realY();

            double val = 0.210461*(Math.pow(moveY,3))+0.953735*(Math.pow(moveY,2))+3.553*(moveY)+11.66231;
            return (int)(83.3333*val);
        }

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(leftSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
        }

        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }


        public class TelemetryArmSlide implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("Arm ", arm.getCurrentPosition());
                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right Slide ", rightSlide.getCurrentPosition());

                telemetry.update();

                return true;
            }
        }
        public Action telemetryArmSlide() {
            return new TelemetryArmSlide();
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


        public class PlaceSample implements Action {
            private boolean wristPlaceSample = false;
            private boolean resetWrist = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Move arm
                moveArm(1650, 1);

                // Once arm reached target, move slides
                if (armReachedTarget(1650, 200)) {
                    moveSlides(2160, 1);
                }

                // Once slide reached target, move wrist
                if (slidesReachedTarget(2160, 20) && !wristPlaceSample) {
                    timer.reset();
                    wrist.PlaceSample();
                    wristPlaceSample = true;
                }

                // Once wrist is moving and timer has reached seconds, open claw
                if (wristPlaceSample && timer.seconds() > 0.8 && !resetWrist) {
                    timer.reset();
                    claw.setPosition(0);
                    resetWrist = true;
                }

                // Once claw is opened and timer reached target, reset wrist
                if (resetWrist && timer.seconds() > 0.5) {
                    wrist.middleWrist();
                    return false;
                }

                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right slide ", rightSlide.getCurrentPosition());
                telemetry.addData("wristPlaceSample ", wristPlaceSample);
                telemetry.addData("resetWrist ", resetWrist);
                telemetry.addData("Timer ", timer.seconds());

                telemetry.update();

                return true;
            }
        }
        public Action placeSample() {
            return new PlaceSample();
        }

        public class GrabSample2 implements Action {
            private boolean closeClaw = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveSlides(1260, 1);
                wrist.PickUp90();

                if (slidesReachedTarget(1260, 10) && !closeClaw) {
                    timer.reset();
                    claw.setPosition(0.5);
                    closeClaw = true;
                }

                if (closeClaw && timer.seconds() > 0.5) {
                    return false;
                }

                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right Slide ", rightSlide.getCurrentPosition());
                telemetry.addData("Close Claw ", closeClaw);
                telemetry.addData("Timer ", timer.seconds());

                telemetry.update();

                return true;
            }
        }
        public Action grabSample2() { return new GrabSample2();}

        public class GrabSample3 implements Action {
            private boolean closeClaw = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveSlides(1200, 1);
                wrist.PickUp90();

                if (slidesReachedTarget(1200, 10) && !closeClaw) {
                    timer.reset();
                    claw.setPosition(0.5);
                    closeClaw = true;
                }

                if (closeClaw && timer.seconds() > 0.5) {
                    return false;
                }

                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right Slide ", rightSlide.getCurrentPosition());
                telemetry.addData("Close Claw ", closeClaw);
                telemetry.addData("Timer ", timer.seconds());

                telemetry.update();

                return true;
            }
        }
        public Action grabSample3() { return new GrabSample3();}

        public class GrabSample4 implements Action {
            private boolean closeClaw = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveSlides(1260, 1);
                wrist.PickUp45Right();

                if (slidesReachedTarget(1260, 10) && !closeClaw) {
                    closeClaw = true;
                    claw.setPosition(0.5);
                    timer.reset();
                }

                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right Slide ", rightSlide.getCurrentPosition());
                telemetry.addData("Close Claw ", closeClaw);
                telemetry.addData("Timer ", timer.seconds());

                telemetry.update();

                return !closeClaw || !(timer.seconds() > 0.5);
            }
        }
        public Action grabSample4() { return new GrabSample4();}

        public class GrabSample5 implements Action {
            private boolean closeClaw = false;
            boolean isExtended = false;
            int slidesTargetPosition = extendedSearchVal();
            int maxAngle = Math.max(Math.max(count0, count90), Math.max(count45l, count45r));

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.moveWristDown();
                if (!isExtended) {
                    claw.setPosition(0.0);
                    moveSlides(extendedSearchVal(), 1);
                    slidesTargetPosition = extendedSearchVal();
                    isExtended = true;
                }




                if (slidesReachedTarget(slidesTargetPosition, 10) && !closeClaw) {
                    closeClaw = true;
                    claw.setPosition(0.5);
                    timer.reset();
                }


                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right Slide ", rightSlide.getCurrentPosition());
                telemetry.addData("Close Claw ", closeClaw);
                telemetry.addData("CalVal INT", slidesTargetPosition);
                telemetry.addData("CalVal ACTUAL", extendedSearchVal());
                telemetry.addData("Timer ", timer.seconds());

                telemetry.update();

                //this need to be figured out
//                if (!closeClaw || !(timer.seconds() > 1)) {
//                    if(maxAngle == count0){
//                        wrist.PickUp90();
//                    }
//                }


                return !closeClaw || !(timer.seconds() > 1);
            }
        }
        public Action grabSample5() { return new GrabSample5();}

        public class ResetAfterPlace implements Action {
            private boolean closeClaw = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveSlides(1150, 1);

                if (slidesReachedTarget(1150, 10) && !closeClaw) {
                    timer.reset();
                    claw.setPosition(0.5);
                    closeClaw = true;
                }

                if (closeClaw && timer.seconds() > 0.5) {
                    return false;
                }

                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right Slide ", rightSlide.getCurrentPosition());
                telemetry.addData("Close Claw ", closeClaw);
                telemetry.addData("Timer ", timer.seconds());

                telemetry.update();

                return true;
            }
        }
        public Action resetAfterPlace() { return new ResetAfterPlace();}

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

                if (resetArm && timer.seconds() > 1.2) {
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

        public class WristUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.Up();

                return false;
            }
        }
        public Action wristUp() { return new WristUp(); }

        public class StopRobot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                return false;
            }
        }
        public Action stopRobot() { return new StopRobot(); }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0.5);

                return false;
            }
        }
        public Action closeClaw() { return new CloseClaw(); }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-55, -55, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder placeSample1 = drive.actionBuilder(new Pose2d(-37, -62, Math.toRadians(0)))
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180));

        TrajectoryActionBuilder grabSample2 = placeSample1.endTrajectory().fresh()
                .setTangent(Math.toRadians(43))
                .lineToYLinearHeading(-48, Math.toRadians(90));

        TrajectoryActionBuilder placeSample2 = grabSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(223))
                .lineToYLinearHeading(-55, Math.toRadians(45));

        TrajectoryActionBuilder grabSample3 = placeSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(122))
                .lineToYLinearHeading(-48, Math.toRadians(90));

        TrajectoryActionBuilder placeSample3 = grabSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(302))
                .lineToYLinearHeading(-55, Math.toRadians(45));

        TrajectoryActionBuilder grabSample4 = placeSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(98))
                .lineToYLinearHeading(-46, Math.toRadians(120));

        TrajectoryActionBuilder placeSample4 = grabSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(278))
                .lineToYLinearHeading(-55, Math.toRadians(45));

        TrajectoryActionBuilder grabSample5 = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(0)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25, -9, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder placeSample5 = grabSample5.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(270));

        TrajectoryActionBuilder grabSample6 = placeSample5.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25, -5, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder placeSample6 = grabSample6.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(270));
        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
//                        new ParallelAction(
//                            grabSample5.build(),
//                            armslidesclaw.resetArm(),
//                            armslidesclaw.resetSlides()
//                        ),
                        armslidesclaw.alignRobot(),
                        armslidesclaw.stopRobot(),
                        armslidesclaw.grabSample5()

//                        placeSample5.build(),
//                        grabSample6.build(),
//                        placeSample6.build()
                )
        );
    }
}