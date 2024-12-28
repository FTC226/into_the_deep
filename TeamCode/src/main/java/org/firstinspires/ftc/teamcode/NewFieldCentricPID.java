package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp (name = "Field Centric PID")
@Config
//@Disabled

public class NewFieldCentricPID extends OpMode {

    // Motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor leftSlide, rightSlide;
    public DcMotorEx arm;
    public Servo leftClaw, rightClaw, claw;




    // IMU
    public IMU imu;
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;

    // Gamepad
    public double leftStickY, leftStickX, rightStickX;

    public boolean switchMode = true;

    // PID
    public static double Kp = 0.5;
    public static double Ki = 0;
    public static double Kd = 0.1;

    public double targetYaw;
    double integralSum = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    // FTC Dashboard

    @Override
    public void init() {
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

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");


        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClaw = hardwareMap.get(Servo.class, "leftServo");
        rightClaw = hardwareMap.get(Servo.class, "rightServo");
        claw = hardwareMap.get(Servo.class, "clawServo");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void loop() {
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        // Yaw Velocity
        double zAccel = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

        // Robot Orientation
        robotOrientation = imu.getRobotYawPitchRollAngles();
        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        if (gamepad1.x) {
            imu.resetYaw();
        }

        // Field Centric Calculations
        double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
        double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);

        frontLeft.setPower((rotY + rotX + rightStickX) / denominator);
        frontRight.setPower((rotY - rotX - rightStickX) / denominator);
        backLeft.setPower((rotY - rotX + rightStickX) / denominator);
        backRight.setPower((rotY + rotX - rightStickX) / denominator);


        if (gamepad2.back) {
            switchMode = !switchMode;
        }



        if(gamepad2.dpad_up && switchMode) {
            wristDown();
            placeSample(); //All Sets for placing sample
        } else if (gamepad2.dpad_up) {
            wristPickUpSpecimen();
            pickupSpecimen();
        }

        if (gamepad2.dpad_down) {
            wristDown();
            resetAction(); //Ready to pick up other samples
        }

        if (gamepad2.dpad_right) {
            if (isWristUp()) {
                wristDown();
            } else {
                wristUp();
            }
            openClaw();
            pickupSample();
        }

        if (gamepad2.dpad_left) {
            closeClaw();
            wristUp();
            resetSlides();
        }

        if (gamepad2.right_stick_button) {
            placeSpecimen();
        }

        if(gamepad2.right_bumper) {
            openClaw();
        }
        if(gamepad2.left_bumper) {
            closeClaw();
        }

        if(gamepad2.y) {
            wristPickUp0();
        }
        if(gamepad2.x) {
            wristPickUp45();
        }
        if(gamepad2.a) {
            wristPickUp90();
        }
        if(gamepad2.b) {
            wristUp();
        }


        if(gamepad2.right_trigger > 0.3) {
            resetEncoderSlides();
        }
        if(gamepad2.left_trigger > 0.3) {
            resetArm();
        }


        if(switchMode) {
            telemetry.addData("MODE", "SAMPLE");
        } else {
            telemetry.addData("MODE", "SPECIMEN");
        }
        telemetry.addData("Yaw: ", Math.toDegrees(robotYaw));
        telemetry.addData("Yaw Acceleration", zAccel);
        telemetry.addData("LeftSlide", leftSlide.getCurrentPosition());
        telemetry.addData("RightSlide", rightSlide.getCurrentPosition());
        telemetry.addData("Arm", arm.getCurrentPosition());
        telemetry.addData("Left Claw", leftClaw.getPosition());
        telemetry.addData("Right Claw", rightClaw.getPosition());

        telemetry.update();
    }

    public boolean slidesReachedTarget(int targetSlides, int threshold) {
        return Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
    }

    public boolean armReachedTarget(int targetArm, int threshold) {
        return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
    }
    public boolean isWristUp() {
        if (leftClaw.getPosition() != -0.248 && rightClaw.getPosition() != 0.556) {
            return false;
        }
        return true;
    }

    public void wristUp() {
        leftClaw.setPosition(-0.248);
        rightClaw.setPosition(0.556);
    }

    public void wristDown() {
        leftClaw.setPosition(0.676);
        rightClaw.setPosition(0.218);
    }
    public void wristPickUp0() {
        leftClaw.setPosition(0.676);
        rightClaw.setPosition(0.218);
    }

    public void wristPickUp45() {
        leftClaw.setPosition(0.72);
        rightClaw.setPosition(0.34);
    }

    public void wristPickUp90() {
        leftClaw.setPosition(0.346);
        rightClaw.setPosition(0.102);
    }
    public void wristPlaceSpecimen() {
        leftClaw.setPosition(0.296);
        rightClaw.setPosition(0.5);
    }

    public void wristPickUpSpecimen() {
        leftClaw.setPosition(0.442);
        rightClaw.setPosition(0.364);
    }

    public void closeClaw() {
        claw.setPosition(1);
    }public void openClaw() {
        claw.setPosition(-1);
    }

    public void placeSample() {

        armPlacingSample();
        if(armReachedTarget(1600, 50)) {
            leftSlide.setTargetPosition(2125);
            rightSlide.setTargetPosition(2125);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }
        if (slidesReachedTarget(2150, 50)) {
            wristUp();
        }
    }

    public void placeSpecimen() {
        wristPlaceSpecimen();
        arm.setTargetPosition(1650);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.8);
        if(armReachedTarget(1650, 50)) {
            leftSlide.setTargetPosition(1000);
            rightSlide.setTargetPosition(1000);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }
    }

    public void pickupSample() {
        leftSlide.setTargetPosition(1100);
        rightSlide.setTargetPosition(1100);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }
    public void pickupSpecimen() {
        arm.setTargetPosition(700);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.8);
    }

    public void resetSlides() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }
    public void resetEncoderSlides() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
        if (slidesReachedTarget(0, 100)) {
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetArm() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.8);
        if (armReachedTarget(0, 100)) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void armPlacingSample() {
        arm.setTargetPosition(1600);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.8);
        if (armReachedTarget(0, 100)) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetAction() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
        if(slidesReachedTarget(0, 300)) {
            resetArm();
        }
    }


}