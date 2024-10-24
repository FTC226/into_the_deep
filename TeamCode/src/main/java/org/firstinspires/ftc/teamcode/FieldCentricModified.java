package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class FieldCentricModified {
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public double frontLeftScalar; //set values based on Encoder Test
    public double frontRightScalar; //set values based on Encoder Test
    public double backLeftScalar; //set values based on Encoder Test
    public double backRightScalar; //set values based on Encoder Test
    // IMU
    public IMU imu;
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;

    // Gamepad
    public double leftStickX, leftStickY, rightStickX;
    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        // Change to Control Hub Orientation
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        imu.resetYaw();
    }

    public void fieldCentric(double leftStickX, double leftStickY, double rightStickX, boolean resetYaw) {

        // IMU Input
        robotOrientation = imu.getRobotYawPitchRollAngles();
        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        // Rotation Calculations
        double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
        double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);

        rotX *= 1.3;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);
        double frontLeftPower = (rotY + rotX + rightStickX) / denominator;
        double frontRightPower = (rotY - rotX - rightStickX) / denominator;
        double backLeftPower = (rotY - rotX + rightStickX) / denominator;
        double backRightPower = (rotY + rotX - rightStickX) / denominator;

        frontLeft.setPower(frontLeftScalar * frontLeftPower);
        frontRight.setPower(frontRightScalar * frontRightPower);
        backLeft.setPower(backLeftScalar * backLeftPower);
        backRight.setPower(backRightScalar * backRightPower);

        // Reset IMU
        if (resetYaw) { imu.resetYaw(); }
    }

    public double readIMU() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
