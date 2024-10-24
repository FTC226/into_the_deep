// https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
// https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp (name = "Jadon Robot Test")
public class JadonRobotTest extends OpMode {

    // Motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    // IMU
    public IMU imu;
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;

    // Gamepad
    public double leftStickY, leftStickX, rightStickX;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Change to Control Hub Orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Test Joysticks
        // testJoysticks();

        // Test Wheels
        // testWheels();

        // Test IMU
        // robotCentric();
        // readIMU();

        // Test Field-Centric
        fieldCentric();
        readIMU();
    }

    public void readIMU() {
        robotOrientation = imu.getRobotYawPitchRollAngles();
        robotYaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        if (gamepad1.x) {
            imu.resetYaw();
        }

        telemetry.addData("Robot Yaw: ", robotYaw);
        telemetry.update();
    }

    public void fieldCentric() {
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        robotOrientation = imu.getRobotYawPitchRollAngles();
        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
        double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);

        // double rotY = gamepad1.left_stick_y * Math.cos(robotYaw) + gamepad1.left_stick_x * Math.sin(robotYaw);
        // double rotX = -gamepad1.left_stick_y * Math.sin(robotYaw) + gamepad1.left_stick_x * Math.cos(robotYaw);
        // double rightStickX = gamepad1.right_stick_x;

        rotX *= 1.3;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);
        double frontLeftPower = (rotY + rotX + rightStickX) / denominator;
        double frontRightPower = (rotY - rotX - rightStickX) / denominator;
        double backLeftPower = (rotY - rotX + rightStickX) / denominator;
        double backRightPower = (rotY + rotX - rightStickX) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void testJoysticks() {
        telemetry.addData("Left stick Y: ", gamepad1.left_stick_y);
        telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
        telemetry.addData("Right stick X:", gamepad1.right_stick_x);
        telemetry.update();
    }

    public void testWheels() {
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        if (leftStickY > 0.1) {
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
        } else if (leftStickY < -0.1) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(-0.5);
        } else if (leftStickX > 0.1) {
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        } else if (leftStickX < -0.1) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
        } else if (rightStickX > 0.1) {
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
        } else if (rightStickX < -0.1) {
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    public void robotCentric() {
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x * 1.1;
        rightStickX = gamepad1.right_stick_x;

        frontLeft.setPower(leftStickY + leftStickX + rightStickX);
        frontRight.setPower(leftStickY - leftStickX - rightStickX);
        backLeft.setPower(leftStickY - leftStickX + rightStickX);
        backRight.setPower(leftStickY + leftStickX - rightStickX);
    }
}
