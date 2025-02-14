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

        private ElapsedTime timer = new ElapsedTime();

        private double xAlign = 0;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        public ArmSlidesClaw(HardwareMap hardwareMap) {

            wrist.init();
            camera.init();
            wrist.middleWrist();

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

//            moveArm(800, 1);
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
            double inches = 2.3027*camera.realX();
            RealYValue = camera.realY();
            RealAngleValue = camera.realAngle();

            return 0.89887640449*inches;
        }

        public double angleOrientation(){
            if (RealYValue != -90) {
                return (0.00377778*(RealYValue)+0.16);
            } else {
                return (0.00377778*(Math.abs(RealYValue))+0.16);
            }
        }

        public int extendedSearchVal(){
            double moveX = 0;
            double moveY = RealYValue;


            double val = 0.210461*(Math.pow(moveY,3))+0.953735*(Math.pow(moveY,2))+3.553*(moveY)+12.66231;
            return (int)(83.3333*val);
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
                    claw.setPosition(0);
                    timer.reset();
                    isReset = true;
                }
                xAlign = xValue;

                telemetry.addData("xAlign", xAlign);
                telemetry.addData("xValue ", xValue);
                telemetry.addData("realX ", camera.realX());

                telemetry.update();


                return !(timer.seconds() > 0.5);
            }
        }
        public Action newRobotAlign() { return new NewRobotAlign();}

        // Getter Method
        public double getXAlign() { return xAlign; }


        public class GrabSample5 implements Action {
            private boolean closeClaw = false;
            boolean isExtended = false;
            int slidesTargetPosition = extendedSearchVal();
            boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!isReset) {
                    timer.reset();
                    isReset = true;
                }

                if (!isExtended && timer.seconds() > 0.5) {
                    claw.setPosition(0.0);
                    moveSlides(extendedSearchVal(), 1);
                    wrist.setRotateServo(angleOrientation());
                    wrist.moveWristDown();
                    slidesTargetPosition = extendedSearchVal();
                    isExtended = true;
                }



                if (slidesReachedTarget(slidesTargetPosition, 50) && !closeClaw) {
                    closeClaw = true;
                    claw.setPosition(0.5);
                    timer.reset();
                }

                return !closeClaw || !(timer.seconds() > 1.5);
            }
        }
        public Action grabSample5() { return new GrabSample5();}

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

                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right slide ", rightSlide.getCurrentPosition());
                telemetry.addData("Timer ", timer.seconds());
                telemetry.addData("RESETAFTERPLACE", null);

                telemetry.update();

                return true;
            }
        }
        public Action resetAfterSubmersible() { return new ResetAfterSubmersible();}

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
                moveArm(1650, 1);

                if (!wristSample) {
                    wrist.PlaceSample();
                    claw.setPosition(0.5);
                }

                // Once arm reached target, move slides
                if (armReachedTarget(1650, 800)) {
                    moveSlides(2130, 1);
                    wristSample = true;
                }

                // Once slide reached target, move wrist
                if (slidesReachedTarget(2130, 20) && !wristPlaceSample) {
                    timer.reset();
                    wristPlaceSample = true;
                }

                // Once wrist is moving and timer has reached seconds, open claw
                if (wristPlaceSample && timer.seconds() > 0.2 && !resetWrist) {
                    timer.reset();
                    claw.setPosition(0);
                    resetWrist = true;
                }

                // Once claw is opened and timer reached target, reset wrist
                if (resetWrist && timer.seconds() > 0.5) {
                    wrist.PickUp0Auto();
                    return false;
                }

                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right slide ", rightSlide.getCurrentPosition());
                telemetry.addData("wristPlaceSample ", wristPlaceSample);
                telemetry.addData("resetWrist ", resetWrist);
                telemetry.addData("Timer ", timer.seconds());
                telemetry.addData("PLACESAMPLE", null);

                telemetry.update();

                return true;
            }
        }
        public Action placeSample() {
            return new PlaceSample();
        }

        public class ResetAfterPlace implements Action {
            private boolean slidesMoveDownAfterSample = false;
            private boolean resetArm = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!slidesMoveDownAfterSample) {
                    moveSlides(800, 1);
                }

                if (slidesReachedTarget(800, 500) && !slidesMoveDownAfterSample) {
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

                if (resetArm && timer.seconds() > 1) {
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                }

                return true;
            }
        }
        public Action resetAfterPlace() { return new ResetAfterPlace();}

        public class GrabSample2 implements Action {
            private boolean extendSlidesSample = false;
            private boolean closeClaw = false;
            private boolean resetSlides = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!extendSlidesSample) {
                    moveSlides(1260, 1);
                }

                if (slidesReachedTarget(1260, 15) && !closeClaw) {
                    timer.reset();
                    claw.setPosition(0.5);
                    closeClaw = true;
                    extendSlidesSample = true;
                }

                if (extendSlidesSample && closeClaw && timer.seconds() > 0.5) {
                    moveSlides(400, 1);
                    resetSlides = true;
                }

                if (resetSlides) {
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
            private boolean extendSlidesSample = false;
            private boolean closeClaw = false;
            private boolean resetSlides = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!extendSlidesSample) {
                    moveSlides(1200, 1);
                }

                if (slidesReachedTarget(1200, 15) && !closeClaw) {
                    timer.reset();
                    claw.setPosition(0.5);
                    closeClaw = true;
                    extendSlidesSample = true;
                }

                if (extendSlidesSample && closeClaw && timer.seconds() > 0.5) {
                    moveSlides(400, 1);
                    resetSlides = true;
                }

                if (resetSlides) {
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
            private boolean extendSlidesSample = false;
            private boolean closeClaw = false;
            private boolean resetSlides = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!extendSlidesSample) {
                    moveSlides(1300, 1);
                    wrist.PickUp45Right();
                }

                if (slidesReachedTarget(1300, 25) && !closeClaw) {
                    timer.reset();
                    claw.setPosition(0.5);
                    closeClaw = true;
                    extendSlidesSample = true;
                }

                if (extendSlidesSample && closeClaw && timer.seconds() > 0.5) {
                    moveSlides(400, 1);
                    wrist.PickUp0Auto();
                    resetSlides = true;
                }

                if (resetSlides) {
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

                if (resetArm && timer.seconds() > 1) {
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
        Pose2d initialPose = new Pose2d(-25, -9, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        telemetry.addData("real_area: ", camera.getRealArea());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Vector2d grabSamplePose = new Vector2d(-25, -9-armslidesclaw.realXtoMM()); // + 9

            TrajectoryActionBuilder alignRobot = drive.actionBuilder(initialPose)
                    .strafeToConstantHeading(grabSamplePose);

            Actions.runBlocking(
                    new SequentialAction(
                            armslidesclaw.newRobotAlign(),
                            alignRobot.build(),
                            armslidesclaw.stopRobot(),
                            armslidesclaw.grabSample5(),
                            armslidesclaw.resetAfterSubmersible(),

                            armslidesclaw.placeSample(),
                            armslidesclaw.resetAfterPlace()
                    )
            );
        }

//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                placeSample1.build(),
//                                armslidesclaw.placeSample()
//                        ),
//                        new ParallelAction(
//                                grabSample2.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetAfterPlace(),
//                                        armslidesclaw.grabSample2()
//                                )
//                        ),
//                        new ParallelAction(
//                                placeSample2.build(),
//                                armslidesclaw.placeSample()
//                        ),
//                        new ParallelAction(
//                                grabSample3.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetAfterPlace(),
//                                        armslidesclaw.grabSample3()
//                                )
//                        ),
//                        new ParallelAction(
//                                placeSample3.build(),
//                                armslidesclaw.placeSample()
//                        ),
//                        new ParallelAction(
//                                grabSample4.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetAfterPlace(),
//                                        armslidesclaw.grabSample4()
//                                )
//                        ),
//                        armslidesclaw.grabSample4(),
//                        new ParallelAction(
//                                placeSample4.build(),
//                                armslidesclaw.placeSample()
//                        ),
//                        grabSample5.build(),
//                        placeSample5.build(),
//                        grabSample6.build(),
//                        placeSample6.build()
//                )
//        );

//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                placeSample1.build(),
//                                armslidesclaw.placeSample()
//                        ),
//                        new ParallelAction(
//                                grabSample2.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetSlides(),
//                                        armslidesclaw.resetArm()
//                                )
//                        ),
//                        armslidesclaw.grabSample2(),
//                        new ParallelAction(
//                                placeSample2.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetSlides(),
//                                        armslidesclaw.placeSample()
//                                )
//                        ),
//                        new ParallelAction(
//                                grabSample3.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetSlides(),
//                                        armslidesclaw.resetArm()
//                                )
//                        ),
//                        armslidesclaw.grabSample3(),
//                        new ParallelAction(
//                                placeSample3.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetSlides(),
//                                        armslidesclaw.placeSample()
//                                )
//                        ),
//                        new ParallelAction(
//                                grabSample4.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetSlides(),
//                                        armslidesclaw.resetArm()
//                                )
//                        ),
//                        armslidesclaw.grabSample4(),
//                        new ParallelAction(
//                                placeSample4.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetSlides(),
//                                        armslidesclaw.placeSample()
//                                )
//                        ),
//                        grabSample5.build(),
//                        placeSample5.build(),
//                        grabSample6.build(),
//                        placeSample6.build()
//                )
//        );

        /*
        Actions.runBlocking(
                new SequentialAction(
                        placeSample1.build(),
                        grabSample2.build(),
                        placeSample2.build(),
                        grabSample3.build(),
                        placeSample3.build(),
                        grabSample4.build(),
                        placeSample4.build(),
                        grabSample5.build(),
                        placeSample5.build(),
                        grabSample6.build(),
                        placeSample6.build()
                )
        );
        */
    }
}