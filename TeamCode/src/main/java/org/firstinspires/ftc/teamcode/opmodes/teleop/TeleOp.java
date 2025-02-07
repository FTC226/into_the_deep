package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//Want to fix the "can't find symbol" the only thing you need to do is refactor the folder subsystems to another name
import org.firstinspires.ftc.teamcode.subss.Arm;
import org.firstinspires.ftc.teamcode.subss.Camera;
import org.firstinspires.ftc.teamcode.subss.Claw;
import org.firstinspires.ftc.teamcode.subss.Slides;
import org.firstinspires.ftc.teamcode.subss.Wrist;


import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "InsiredAwardTeleop")
@Config

public class TeleOp extends OpMode {

    //Subsystems
    Arm arm = new Arm(this, 1);
    Slides slides = new Slides(this);
    Wrist wrist = new Wrist(this);
    Claw claw = new Claw(this);
    Camera camera = new Camera(this);
    private OpenCvCamera webcam;

    public double speed = 1.0;
    public double angle;
    public Point center;

    // Motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    // IMU
    public IMU imu;
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;


    // Gamepad
    public double leftStickY, leftStickX, rightStickX;


    //Robot Yaw
    public double targetYaw;



    //Timer




    @Override
    public void init() {

        arm.init();
        slides.init();
        wrist.init();
        claw.init();
        camera.init();

        FtcDashboard dashboard = FtcDashboard.getInstance();



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


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Initialize the pipeline

        webcam.setPipeline(camera);




        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);

                dashboard.startCameraStream(webcam, 50);
                telemetry.addData("Status", "Camera started");


            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open with error code: " + errorCode);
                telemetry.update();
            }
        });

    }
    @Override
    public void loop() {

        //Gamepad JoyStick
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;
        double zAccel = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);
        if (gamepad1.x) {
            imu.resetYaw();
        }
        // Field Centric Calculations
        double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
        double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);

        if (gamepad1.right_trigger > 0.8) {
            speed = 0.25;
        } else {
            speed = 1.0;
        }


        wrist.Middle();
        search();
//        extendedSearch();


        slides.resetEncoder();


//
//        if (gamepad1.b) {
//        } else {
//            frontLeft.setPower(((rotY + rotX + rightStickX) / denominator) * speed);
//            frontRight.setPower(((rotY - rotX - rightStickX) / denominator) * speed);
//            backLeft.setPower(((rotY - rotX + rightStickX) / denominator) * speed);
//            backRight.setPower(((rotY + rotX - rightStickX) / denominator) * speed);
//        }
//


        if(gamepad1.a){
            camera.setColor("yellow");
        }

        if(gamepad1.a){
            camera.setColor("blue");
        }

        if(gamepad1.dpad_up){
            camera.setColor("red");
        }



        if(gamepad2.dpad_up) {
            placeSample();
        }


        if (gamepad2.dpad_down) {
            resetAction();
        }


        if (gamepad2.dpad_right) {
            if (wrist.isUp()) {
                wrist.Down();
            } else {
                wrist.Up();
            }
            pickupSample();
        }


        if (gamepad2.dpad_left) {
            resetSlides();
        }


        if (gamepad2.right_stick_button) {
            arm.pickUpSpecimen();
        }


        if(gamepad2.right_bumper) {
            if (slidesReachedTarget(2125, 50)) {
                claw.openClaw();
                wrist.PlaceSample();
                wrist.ReadyPlaceSample();
            }
            else {
                claw.openClaw();
            }
        }


        if(gamepad2.left_bumper) {
            claw.closeClaw();
        }



        if(gamepad2.y) {
            wrist.PickUp0();
        }


        if(gamepad2.x) {
            placeSampleLow();
        }


        if(gamepad2.a) {
            wrist.PickUp90();
        }


        if(gamepad2.b) {
            wrist.Up();
        }


        if(gamepad2.left_trigger > 0.3) {
            armHang();
        }
        if(gamepad2.right_trigger > 0.3) {
            slides.hangExtend();
        }


        telemetry.addData("Angle", camera.realAngle());
        telemetry.addData("Center X", camera.realX());
        telemetry.addData("Center Y", camera.realY());
        telemetry.addData("Yaw: ", Math.toDegrees(robotYaw));
        telemetry.addData("Yaw Acceleration", zAccel);
        telemetry.addData("LeftSlide", slides.leftGetCurrentPosition());
        telemetry.addData("RightSlide", slides.rightGetCurrentPosition());
        telemetry.addData("Arm", arm.getCurrentPosition());
        telemetry.update();


    }
    public boolean slidesReachedTarget(int targetSlides, int threshold) {
        return Math.abs(slides.rightGetCurrentPosition() - targetSlides) < threshold && Math.abs(slides.rightGetCurrentPosition() - targetSlides) < threshold;
    }
    public boolean armReachedTarget(int targetArm, int threshold) {
        return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
    }


    public void armHang() {
        arm.readyForHang();
        if (armReachedTarget(500, 100)) {
            slides.hangExtend();
        }
        if (slidesReachedTarget(1100, 50)) {
            arm.moveUp();
        }
    }

    public void extendedSearch(){
        double moveX = 0;
        double moveY = camera.realY();

        double functionVal = 0.237604*(Math.pow(moveY,3))+0.93821*(Math.pow(moveY,2))+3.53806*(moveY)+12.0179;
        slides.move((int)(55.52652208*functionVal));

    }

    public void search() {
        double moveX = camera.realX();
        double moveY = 0;
        boolean angle = camera.bigAngle(); //will always be bigAngle
        boolean angleSet = false;
        boolean centralized = false;

        //move the claw first
//        if (!angle && !angleSet){
//            wrist.PickUp90();
//            angleSet = true;
//        }
//        else if(angle && !angleSet){
//            wrist.PickUp0();
//            moveY+=1;
//            angleSet = true;
//        }

         double magnitude = Math.sqrt(Math.pow(moveX,2)+Math.pow(moveY,2));
         double deltaX = moveX/magnitude;
         double deltaY = moveY/magnitude;
         double denominator = Math.max(Math.abs(moveX), 1);
         double searchVelocity = 0.25;

        frontLeft.setPower(((moveX) / denominator) * searchVelocity);
        frontRight.setPower(((-moveX) / denominator) * searchVelocity);
        backLeft.setPower(((-moveX) / denominator) * searchVelocity);
        backRight.setPower(((moveX) / denominator) * searchVelocity);

//        if(Math.abs(moveX)>0.1 && Math.abs(moveY)>0.1 && !centralized){
//            double magnitude = Math.sqrt(Math.pow(moveX,2)+Math.pow(moveY,2));
//            double deltaX = moveX/magnitude;
//            double deltaY = moveY/magnitude;
//            double denominator = Math.max(Math.abs(deltaX)+Math.abs(deltaY), 1);
//
//            frontLeft.setPower(((moveY + moveX + rightStickX) / denominator) * 0.25);
//            frontRight.setPower(((moveY - moveX - rightStickX) / denominator) * 0.25);
//            backLeft.setPower(((moveY - moveX + rightStickX) / denominator) * 0.25);
//            backRight.setPower(((moveY + moveX - rightStickX) / denominator) * 0.25);
//        } else if (Math.abs(moveX)>0.3 && Math.abs(moveY)>0.3){
//            centralized = true;
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//        }

        //centralize the bot second




    }





//    public void search() {
//        double moveX = camera.realX();
//        double moveY = 0;
//        boolean angle = camera.bigAngle(); //will always be bigAngle
//        boolean angleSet = false;
//        boolean centralized = false;
//
//        moveX = Math.min(1.0, moveX);
////        moveY = Math.min(1.0, moveY);
//        //move the claw first
////        if (!angle && !angleSet){
////            wrist.PickUp90();
////            angleSet = true;
////        }
////        else if(angle && !angleSet){
////            wrist.PickUp0();
////            moveY+=1;
////            angleSet = true;
////        }
//
//        double magnitude = Math.sqrt(Math.pow(moveX,2)+Math.pow(moveY,2));
//        double deltaX = moveX/magnitude;
//        double deltaY = moveY/magnitude;
//        double denominator = Math.max(Math.abs(deltaX)+Math.abs(deltaY), 1);
//
//        frontLeft.setPower(((moveY + moveX + rightStickX) / denominator) * 0.25);
//        frontRight.setPower(((moveY - moveX - rightStickX) / denominator) * 0.25);
//        backLeft.setPower(((moveY - moveX + rightStickX) / denominator) * 0.25);
//        backRight.setPower(((moveY + moveX - rightStickX) / denominator) * 0.25);
//
////        if(Math.abs(moveX)>0.1 && Math.abs(moveY)>0.1 && !centralized){
////            double magnitude = Math.sqrt(Math.pow(moveX,2)+Math.pow(moveY,2));
////            double deltaX = moveX/magnitude;
////            double deltaY = moveY/magnitude;
////            double denominator = Math.max(Math.abs(deltaX)+Math.abs(deltaY), 1);
////
////            frontLeft.setPower(((moveY + moveX + rightStickX) / denominator) * 0.25);
////            frontRight.setPower(((moveY - moveX - rightStickX) / denominator) * 0.25);
////            backLeft.setPower(((moveY - moveX + rightStickX) / denominator) * 0.25);
////            backRight.setPower(((moveY + moveX - rightStickX) / denominator) * 0.25);
////        } else if (Math.abs(moveX)>0.3 && Math.abs(moveY)>0.3){
////            centralized = true;
////            frontLeft.setPower(0);
////            frontRight.setPower(0);
////            backLeft.setPower(0);
////            backRight.setPower(0);
////        }
//
//        //centralize the bot second
//
//
//
//
//    }

    public void placeSample() {
        wrist.ReadyPlaceSample();
        arm.moveUp();
        if(armReachedTarget(1650, 1000)) {
            slides.placeSample();
        }
    }

    public void placeSampleLow() {
        wrist.PlaceSample();
        arm.moveUp();
        if(armReachedTarget(1650, 1000)) {
            slides.placeSampleLow();
        }
    }
    public void placeSpecimen() {
        wrist.Up();
        arm.moveUp();
        if(armReachedTarget(1650, 100)) {
            slides.getReadyPlaceSpecimen();
        }
//        if(slidesReachedTarget(900, 100)) {
//            claw.openClaw();
//        }
    }
    public void pickupSample() {
        if (armReachedTarget(1650, 100)) {
            slides.placeSpecimen();
        } else {
            claw.openClaw();
            slides.pickupSample();
        }
    }

    public void resetSlides() {
        wrist.Up();
        slides.moveToResetPos();
    }


    public void resetAction() {
        wrist.Down();
        slides.moveToResetPos();
        if(slidesReachedTarget(0, 300)) {
            arm.moveDown();
        }
    }



}