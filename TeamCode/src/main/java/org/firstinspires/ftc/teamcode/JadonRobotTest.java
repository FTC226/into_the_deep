// https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
// https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

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
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
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
        // fieldCentric();
    }

    public void readIMU() {
        robotOrientation = imu.getRobotYawPitchRollAngles();
        robotYaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        telemetry.addData("Robot Yaw: ", robotYaw);
        telemetry.update();
    }

    public void testJoysticks() {
        telemetry.addData("Left stick Y: ", gamepad1.left_stick_y);
        telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
        telemetry.update();
    }

    public void testWheels() {
        leftStickY = -gamepad1.left_stick_y;

        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
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

    public void fieldCentric() {
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        robotOrientation = imu.getRobotYawPitchRollAngles();
        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
        double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);

        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);
        double frontLeftPower = (rotY + rotX + rightStickX) / denominator;
        double backLeftPower = (rotY - rotX + rightStickX) / denominator;
        double frontRightPower = (rotY - rotX - rightStickX) / denominator;
        double backRightPower = (rotY + rotX - rightStickX) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
}
