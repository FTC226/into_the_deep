//package org.firstinspires.ftc.teamcode.opmodes.teleop;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
//
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "FOC")
//@Config
//
//public class FOC extends OpMode {
//
//    //Subsystems
//
//
//    // Motors
//    public DcMotor frontLeft, frontRight, backLeft, backRight; //phần dành cho lái
//
//    public DcMotorEx leftSlide, rightSlide;
//
//    public Servo leftSlideServo, rightSlideServo, leftArmUp, rightArmUp, leftArmDown, rightArmDown;
//
///////////////////////////////////////////////////////////
//    // IMU
//    public double speed = 1.0;
//    public IMU imu;
//    public YawPitchRollAngles robotOrientation;
//    public double robotYaw;
//
//    //0-1
//
//    // Gamepad
//    public double leftStickY, leftStickX, rightStickX;
//
//
//    //Robot Yaw
//    public double targetYaw;
///////////////////////////////////////////////////////////
//
//
//    @Override
//    public void init() {
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//
////Chỉnh hướng cho slides
//        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
//        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
//
//        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD); //need to change
//        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE); // need to change
//
//        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
///// Phần cần điều chỉnh lại hướng
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setDirection(DcMotor.Direction.FORWARD);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
//                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//        imu.initialize(parameters);
//        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//    }
//    @Override
//    public void loop() {
//
//        //Gamepad JoyStick
//        leftStickY = -gamepad1.left_stick_y;
//        leftStickX = gamepad1.left_stick_x;
//        rightStickX = gamepad1.right_stick_x;
//        double zAccel = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
//        robotOrientation = imu.getRobotYawPitchRollAngles();
//        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);
//
//        if (gamepad1.x) {
//            imu.resetYaw();
//        }
//        // Field Centric Calculations
//        double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
//        double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);
//
//
//        if (gamepad1.right_trigger > 0.8) {
//            speed = 0.25;
//        } else {
//            speed = 1.0;
//        }
//
//
//        frontLeft.setPower(((rotY + rotX + rightStickX) / denominator) * speed);
//        frontRight.setPower(((rotY - rotX - rightStickX) / denominator) * speed);
//        backLeft.setPower(((rotY - rotX + rightStickX) / denominator) * speed);
//        backRight.setPower(((rotY + rotX - rightStickX) / denominator) * speed);
//
//
//        if(gamepad2.dpad_up) {
//            moveSlidesUp();
//        }
//
//        if(gamepad2.dpad_down) {
//            moveSlidesDown();
//        }
//
//        if(gamepad2.dpad_left) {
//            leftSlideServo.setPosition(0);
//            rightSlideServo.setPosition(0);
//        }
//
//    }
////preset
//    public void moveSlidesUp() {
//        moveSlides(900, 5000);
//    }
//
//    public void moveSlidesDown() {
//        moveSlides(0, 5000);
//    }
//
//    public void moveSlides(int targetPosition, double velocity) {
//        leftSlide.setTargetPosition(targetPosition);
//        rightSlide.setTargetPosition(targetPosition);
//        leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        leftSlide.setVelocity(velocity);
//        rightSlide.setVelocity(velocity);
//    }
//}