package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import androidx.annotation.NonNull;

public class Drive { //subset
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public double botHeading;

    public IMU imu;
    HardwareMap hwMap;

    // IMU
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;

    // PID
    public static double Kp = 0.5;
    public static double Ki = 0;
    public static double Kd = 0.1;

    public double targetYaw;
    double integralSum = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();
    Telemetry Telem;


    public Drive(HardwareMap ahwMap, Telemetry telem) {

        hwMap = ahwMap;
        Telem = telem;

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");


        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       /* linSlide.setDirection(DcMotor.Direction.REVERSE);
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        imu.resetYaw();
        targetYaw = 0;
        integralSum = 0;
        lastError = 0;
        botHeading = 0;

    }

    public void driveFC(double x, double y, double rx, boolean resetIMU){
        robotOrientation = imu.getRobotYawPitchRollAngles();

        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-robotYaw) - y * Math.sin(-robotYaw);
        double rotY = x * Math.sin(-robotYaw) + y * Math.cos(-robotYaw);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        //  || Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate) > 1
        if (Math.abs(rx) > 0.1) {
            targetYaw = robotYaw;
            lastError = 0;
        }

        if (resetIMU) {
            imu.resetYaw();
            targetYaw = 0;
            integralSum = 0;
            lastError = 0;
        }

        // PID Calculations
        double error = angleWrap(targetYaw - robotYaw);

        if (Math.abs(error) < Math.toRadians(2)) { // 2° tolerance
            error = 0;
        }

        // Compute PID Terms
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double correction = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        double rotationPower = Math.abs(rx) > 0.1 ? rx : -correction;

        frontLeft.setPower((rotY + rotX + rotationPower) / denominator);
        frontRight.setPower((rotY - rotX - rotationPower) / denominator);
        backLeft.setPower((rotY - rotX + rotationPower) / denominator);
        backRight.setPower((rotY + rotX - rotationPower) / denominator);

        lastError = error;
        timer.reset();

//        // FTC Dashboard
        Telem.addData("Target: ", Math.toDegrees(targetYaw));
        Telem.addData("Actual: ", Math.toDegrees(robotYaw));
        Telem.addData("Error: ", Math.toDegrees(error));
        Telem.addData("Yaw Acceleration", Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));
        Telem.update();
    }

    // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
}