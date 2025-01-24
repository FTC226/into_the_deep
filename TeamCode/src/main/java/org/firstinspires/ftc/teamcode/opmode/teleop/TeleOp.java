package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//Want to fix the "can't find symbol" the only thing you need to do is refactor the folder subsystems to another name
import org.firstinspires.ftc.teamcode.subss.Arm;
import org.firstinspires.ftc.teamcode.subss.Camera;
import org.firstinspires.ftc.teamcode.subss.Claw;
import org.firstinspires.ftc.teamcode.subss.Slides;
import org.firstinspires.ftc.teamcode.subss.Wrist;


import org.opencv.core.*;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "InsiredAwardTeleop")
@Config

public class TeleOp extends OpMode {

    //Subsystems
    Arm arm = new Arm(this, 1);
    Slides slides = new Slides(this);
    Wrist wrist = new Wrist(this);
    Claw claw = new Claw(this);
    Camera camera = new Camera();

    public boolean switchMode = true;

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
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {

        arm.init();
        slides.init();
        wrist.init();
        claw.init();


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

        if(gamepad1.b){ // moving to centralize the claw

            angle = camera.Angle();
            center = camera.Center();

            double x = center.x;
            double y = center.y;
            if(Math.abs(x-320)>20 && Math.abs(y-180)>20){//setting the center
                moveRobotCentric(-(x-320)/Math.sqrt((x*x)+(y*y)), -(y-180)/Math.sqrt((x*x)+(y*y)), 0);
            } else if(angle <= 122.5 && angle >= 67.5){
                wrist.PickUp0();
            } else if(angle > 122.5 || angle < 67.5){
                wrist.PickUp90();
            } else {
                claw.openClaw(); //open the claw
            }


        } else{

            frontLeft.setPower(((rotY + rotX + rightStickX) / denominator)*speed);
            frontRight.setPower(((rotY - rotX - rightStickX) / denominator)*speed);
            backLeft.setPower(((rotY - rotX + rightStickX) / denominator)*speed);
            backRight.setPower(((rotY + rotX - rightStickX) / denominator)*speed);

        }


        //Switch Mode
        if (gamepad2.back) {
            switchMode = !switchMode;
        }


        //Gamepad Controll

//        slides.moveSlidesManual(gamepad2.left_stick_y);


        if(gamepad2.dpad_up && switchMode) {
            placeSample();
        } else if (gamepad2.dpad_up) {
            placeSpecimen();
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
            wrist.PickUpSpecimenOp();
        }


        if(gamepad2.right_bumper) {
            claw.openClaw();
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



        if(switchMode) {
            telemetry.addData("MODE", "SAMPLE");
        } else {
            telemetry.addData("MODE", "SPECIMEN");
        }

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
        if (armReachedTarget(500, 50)) {
            slides.hangExtend();
        }
        if (slidesReachedTarget(1100, 50)) {
            arm.moveUp();
        }
    }


    public void placeSample() {
//        wrist.Down();
        wrist.placeSample();
        arm.moveUp();
        if(armReachedTarget(1650, 500)) {
            slides.placeSample();
        }
//        if (slidesReachedTarget(2150, 50)) {
//            wrist.Up();
//        }
    }

    public void placeSampleLow() {
//        wrist.Down();
        wrist.placeSample();
        arm.moveUp();
        if(armReachedTarget(1650, 500)) {
            slides.placeSampleLow();
        }
//        if (slidesReachedTarget(2150, 50)) {
//            wrist.Up();
//        }
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

    public void moveRobotCentric(double x, double y, double rx){
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / d;
        double backLeftPower = (y - x + rx) / d;
        double frontRightPower = (y - x - rx) / d;
        double backRightPower = (y + x - rx) / d;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

}